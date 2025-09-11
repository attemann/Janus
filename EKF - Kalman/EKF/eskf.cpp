
#include "ESKF.h"

// ===== basic vector/quaternion helpers =====
Vec3 ESKF::add(const Vec3&a,const Vec3&b){ return make3(a.x+b.x,a.y+b.y,a.z+b.z); }
Vec3 ESKF::sub(const Vec3&a,const Vec3&b){ return make3(a.x-b.x,a.y-b.y,a.z-b.z); }
Vec3 ESKF::scale(const Vec3&a,float s){ return make3(a.x*s,a.y*s,a.z*s); }
Vec3 ESKF::cross3(const Vec3&a,const Vec3&b){
  return make3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
Quat ESKF::qMul(const Quat& a, const Quat& b){
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}
void ESKF::quatNormalize(Quat& q){
  float n = sqrtf(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  if(n>0){ q.w/=n; q.x/=n; q.y/=n; q.z/=n; } else { q=qIdentity(); }
}
Quat ESKF::fromEuler(float r,float p,float y){
  float cr=cosf(r*0.5f), sr=sinf(r*0.5f);
  float cp=cosf(p*0.5f), sp=sinf(p*0.5f);
  float cy=cosf(y*0.5f), sy=sinf(y*0.5f);
  return {
    cr*cp*cy + sr*sp*sy,
    sr*cp*cy - cr*sp*sy,
    cr*sp*cy + sr*cp*sy,
    cr*cp*sy - sr*sp*cy
  };
}
void ESKF::skew(const Vec3& v, float S[9]){
  S[0]=0;    S[1]=-v.z; S[2]=v.y;
  S[3]=v.z;  S[4]=0;    S[5]=-v.x;
  S[6]=-v.y; S[7]=v.x;  S[8]=0;
}
void ESKF::Rnb(const Quat& q, float R[9]){
  const float w=q.w,x=q.x,y=q.y,z=q.z;
  const float ww=w*w, xx=x*x, yy=y*y, zz=z*z;
  const float wx=w*x, wy=w*y, wz=w*z;
  const float xy=x*y, xz=x*z, yz=y*z;
  // NED <- body
  R[0]=ww+xx-yy-zz; R[1]=2*(xy-wz);   R[2]=2*(xz+wy);
  R[3]=2*(xy+wz);   R[4]=ww-xx+yy-zz;R[5]=2*(yz-wx);
  R[6]=2*(xz-wy);   R[7]=2*(yz+wx);  R[8]=ww-xx-yy+zz;
}
Vec3 ESKF::rotateBodyToNED(const Quat& q, const Vec3& vb){
  float R[9]; Rnb(q,R);
  return make3(
    R[0]*vb.x + R[1]*vb.y + R[2]*vb.z,
    R[3]*vb.x + R[4]*vb.y + R[5]*vb.z,
    R[6]*vb.x + R[7]*vb.y + R[8]*vb.z
  );
}
void ESKF::nedMetersPerDeg(float lat, float& mlat, float& mlon){
  // good within ~10 km: Bowring-ish approximation
  float s = sinf(lat), c = cosf(lat);
  mlat = 111132.92f - 559.82f*cosf(2*lat) + 1.175f*cosf(4*lat);
  mlon = (111412.84f*c) - 93.5f*c*c*c;
}

// ===== matrix helpers (tiny) =====
void ESKF::eye(float* A, int n){ memset(A,0,n*n*sizeof(float)); for(int i=0;i<n;i++) A[i*n+i]=1.0f; }
void ESKF::addInPlace(float* A, const float* B, int n){ for(int i=0;i<n*n;i++) A[i]+=B[i]; }
void ESKF::AtimesB(float* C,const float* A,const float* B,int n,int m,int p){
  // C(n×p)=A(n×m)B(m×p)
  for(int i=0;i<n;i++) for(int j=0;j<p;j++){
    float s=0; for(int k=0;k<m;k++) s+=A[i*m+k]*B[k*p+j];
    C[i*p+j]=s;
  }
}
void ESKF::AtimesBT(float* C,const float* A,const float* B,int n,int m,int p){
  // C(n×p)=A(n×m)B^T (p×m)^T => B(p×m)
  for(int i=0;i<n;i++) for(int j=0;j<p;j++){
    float s=0; for(int k=0;k<m;k++) s+=A[i*m+k]*B[j*m+k];
    C[i*p+j]=s;
  }
}
bool ESKF::invert3(const float A[9], float inv[9]){
  float a=A[0],b=A[1],c=A[2], d=A[3],e=A[4],f=A[5], g=A[6],h=A[7],i=A[8];
  float det = a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
  if(fabsf(det)<1e-9f) return false;
  float id=1.0f/det;
  inv[0]=(e*i-f*h)*id; inv[1]=(c*h-b*i)*id; inv[2]=(b*f-c*e)*id;
  inv[3]=(f*g-d*i)*id; inv[4]=(a*i-c*g)*id; inv[5]=(c*d-a*f)*id;
  inv[6]=(d*h-e*g)*id; inv[7]=(b*g-a*h)*id; inv[8]=(a*e-b*d)*id;
  return true;
}
bool ESKF::solveSymmetric3(const float S[9], const float z[3], float K[9]){
  float Sinv[9]; if(!invert3(S,Sinv)) return false;
  // K = PHt * S^{-1} (caller builds PHt externally; here just return Sinv to reuse)
  memcpy(K, Sinv, 9*sizeof(float));
  return true;
}

// ===== ESKF core =====
ESKF::ESKF(const ESKFConfig& cfg): cfg_(cfg) {
  reset(make3(),make3(),qIdentity(),make3(),make3(), 10.0f);
}
void ESKF::reset(const Vec3& p0,const Vec3& v0,const Quat& q0,const Vec3& bg0,const Vec3& ba0,float Pdiag){
  st_.p=p0; st_.v=v0; st_.q=q0; quatNormalize(st_.q); st_.bg=bg0; st_.ba=ba0;
  eye(st_.P,15);
  for(int i=0;i<15;i++) st_.P[i*15+i]=Pdiag; // big until first GNSS locks it in
}

void ESKF::propagateCov(float dt, const Vec3& omega_b, const Vec3& f_b, const float Rnb_[9]){
  // Error-state ordering: [δp δv δθ δbg δba]
  float F[15*15]; memset(F,0,sizeof(F));
  // d(δp)/dt = δv
  F[0*15 + 3] = 1; F[1*15 + 4] = 1; F[2*15 + 5] = 1;
  // d(δv)/dt = -Rnb*[f_b]_x * δθ - Rnb*δba
  float Sf[9]; skew(f_b,Sf);
  // -Rnb*[f]_x  -> 3x3 block at (δv, δθ)
  for(int r=0;r<3;r++) for(int c=0;c<3;c++){
    // block (3..5, 6..8)
    F[(3+r)*15 + (6+c)] = -(Rnb_[r*3+0]*Sf[0*3+c] + Rnb_[r*3+1]*Sf[1*3+c] + Rnb_[r*3+2]*Sf[2*3+c]);
    // block (3..5, 12..14): -Rnb
    F[(3+r)*15 + (12+c)] = -Rnb_[r*3+c];
  }
  // d(δθ)/dt = -[ω_b]_x δθ - δbg
  float So[9]; skew(omega_b,So);
  for(int r=0;r<3;r++) for(int c=0;c<3;c++){
    F[(6+r)*15 + (6+c)] = -So[r*3+c];
  }
  F[6*15+9] = -1; F[7*15+10]= -1; F[8*15+11]= -1;

  // Discrete Φ ≈ I + F dt; Qd ≈ G*Qc*G^T dt
  float Phi[15*15]; eye(Phi,15);
  for(int i=0;i<15;i++) for(int j=0;j<15;j++) Phi[i*15+j]+=F[i*15+j]*dt;

  // Build Qc (continuous) with blocks for gyro/accel noise & bias RW
  float Qc[15*15]; memset(Qc,0,sizeof(Qc));
  const float sg2 = cfg_.sigma_g*cfg_.sigma_g;
  const float sa2 = cfg_.sigma_a*cfg_.sigma_a;
  const float sbg2= cfg_.sigma_bg*cfg_.sigma_bg;
  const float sba2= cfg_.sigma_ba*cfg_.sigma_ba;

  // process noise affects δv via accel noise -> Rnb * a_noise
  for(int r=0;r<3;r++) for(int c=0;c<3;c++){
    // δv block add: Rnb * sa2 * Rnb^T
    Qc[(3+r)*15 + (3+c)] += (Rnb_[r*3+0]*Rnb_[c*3+0] + Rnb_[r*3+1]*Rnb_[c*3+1] + Rnb_[r*3+2]*Rnb_[c*3+2]) * sa2;
    // δθ block add: gyro noise -> sg2 * I
    Qc[(6+r)*15 + (6+c)] += (r==c? sg2 : 0.0f);
    // δbg block add: sbg2 * I; δba block add: sba2 * I
    Qc[(9+r)*15 + (9+c)]  += (r==c? sbg2: 0.0f);
    Qc[(12+r)*15 + (12+c)]+= (r==c? sba2: 0.0f);
  }

  // P = Φ P Φ^T + Qc*dt
  float tmp[15*15]; AtimesB(tmp, Phi, st_.P, 15,15,15);
  float newP[15*15]; AtimesBT(newP, tmp, Phi, 15,15,15);
  for(int i=0;i<15*15;i++) newP[i]+=Qc[i]*dt;
  memcpy(st_.P, newP, sizeof(newP));
}

void ESKF::predict(float dt, const Vec3& gyro, const Vec3& acc){
  // 1) Nominal state integration
  Vec3 omega = sub(gyro, st_.bg);
  Vec3 f     = sub(acc,  st_.ba);

  // quaternion incremental rotation (small-angle)
  float theta = sqrtf(omega.x*omega.x + omega.y*omega.y + omega.z*omega.z) * dt;
  Quat dq = {1, 0.5f*omega.x*dt, 0.5f*omega.y*dt, 0.5f*omega.z*dt};
  st_.q = qMul(st_.q, dq); quatNormalize(st_.q);

  float Rn[9]; Rnb(st_.q, Rn);
  Vec3 a_ned = rotateBodyToNED(st_.q, f);
  a_ned = add(a_ned, nedGravity(cfg_.g));  // add +g in NED (z positive down)

  st_.v = add(st_.v, scale(a_ned, dt));
  st_.p = add(st_.p, scale(st_.v, dt));

  // 2) Covariance propagation
  propagateCov(dt, omega, f, Rn);
}

void ESKF::injectCorrection(const float dx[15]){
  // δp, δv, δθ, δbg, δba
  st_.p = add(st_.p, make3(dx[0],dx[1],dx[2]));
  st_.v = add(st_.v, make3(dx[3],dx[4],dx[5]));
  // attitude: apply small angle δθ
  Vec3 dth = make3(dx[6],dx[7],dx[8]);
  Quat dq = {1.0f, 0.5f*dth.x, 0.5f*dth.y, 0.5f*dth.z};
  st_.q = qMul(st_.q, dq); quatNormalize(st_.q);
  st_.bg = add(st_.bg, make3(dx[9],dx[10],dx[11]));
  st_.ba = add(st_.ba, make3(dx[12],dx[13],dx[14]));
  // reset error covariance cross-terms if desired (not strictly needed here)
}

void ESKF::updatePositionNED(const Vec3& p_meas, float sigma){
  if(sigma<=0) sigma = cfg_.sigma_gnss_pos;
  // H = [I 0 0 0 0]
  float H[3*15] = {0};
  H[0]=1; H[1*15+1]=1; H[2*15+2]=1;
  float Rm[9]={sigma*sigma,0,0, 0,sigma*sigma,0, 0,0,sigma*sigma};

  // innovation y = z - p
  float y[3] = { p_meas.x - st_.p.x, p_meas.y - st_.p.y, p_meas.z - st_.p.z };

  // S = HPH^T + R
  float HPt[3*15];
  // HPt = H * P (3x15)
  for(int r=0;r<3;r++) for(int c=0;c<15;c++){
    HPt[r*15+c] = H[r*15+0]*st_.P[0*15+c] + H[r*15+1]*st_.P[1*15+c] + H[r*15+2]*st_.P[2*15+c];
  }
  float S[9]; AtimesBT(S, HPt, HPt, 3,15,3);
  for(int i=0;i<9;i++) S[i]+=Rm[i];

  // K = PH^T S^{-1}
  float Sinv[9]; if(!invert3(S,Sinv)) return;
  float PHt[15*3];
  // PHt = P * H^T (15x3) -> same as (HPt)^T
  for(int r=0;r<15;r++) for(int c=0;c<3;c++) PHt[r*3+c] = HPt[c*15+r];

  float K[15*3]; AtimesB(K, PHt, Sinv, 15,3,3);

  // dx = K y
  float dx[15]={0};
  for(int r=0;r<15;r++){
    dx[r] = K[r*3+0]*y[0] + K[r*3+1]*y[1] + K[r*3+2]*y[2];
  }

  // P = (I-KH)P
  float KH[15*15]={0};
  for(int r=0;r<15;r++) for(int c=0;c<15;c++){
    // only first 3 cols of H are nonzero
    KH[r*15+c] = K[r*3+0]*H[0*15+c] + K[r*3+1]*H[1*15+c] + K[r*3+2]*H[2*15+c];
  }
  float I[15*15]; eye(I,15);
  for(int i=0;i<15*15;i++) I[i]-=KH[i];
  float newP[15*15]; AtimesB(newP, I, st_.P, 15,15,15);
  memcpy(st_.P,newP,sizeof(newP));

  injectCorrection(dx);
}

void ESKF::updateVelocityNED(const Vec3& v_meas, float sigma){
  if(sigma<=0) sigma = cfg_.sigma_gnss_vel;
  // H = [0 I 0 0 0]
  float H[3*15]={0}; H[0*15+3]=1; H[1*15+4]=1; H[2*15+5]=1;
  float Rm[9]={sigma*sigma,0,0, 0,sigma*sigma,0, 0,0,sigma*sigma};

  float y[3] = { v_meas.x - st_.v.x, v_meas.y - st_.v.y, v_meas.z - st_.v.z };

  float HPt[3*15];
  for(int r=0;r<3;r++) for(int c=0;c<15;c++){
    HPt[r*15+c] = H[r*15+3]*st_.P[3*15+c] + H[r*15+4]*st_.P[4*15+c] + H[r*15+5]*st_.P[5*15+c];
  }
  float S[9]; AtimesBT(S, HPt, HPt, 3,15,3);
  for(int i=0;i<9;i++) S[i]+=Rm[i];

  float Sinv[9]; if(!invert3(S,Sinv)) return;
  float PHt[15*3];
  for(int r=0;r<15;r++) for(int c=0;c<3;c++) PHt[r*3+c] = HPt[c*15+r];

  float K[15*3]; AtimesB(K, PHt, Sinv, 15,3,3);

  float dx[15]={0};
  for(int r=0;r<15;r++) dx[r] = K[r*3+0]*y[0] + K[r*3+1]*y[1] + K[r*3+2]*y[2];

  float KH[15*15]={0};
  for(int r=0;r<15;r++) for(int c=0;c<15;c++){
    KH[r*15+c] = K[r*3+0]*H[0*15+c] + K[r*3+1]*H[1*15+c] + K[r*3+2]*H[2*15+c];
  }
  float I[15*15]; eye(I,15);
  for(int i=0;i<15*15;i++) I[i]-=KH[i];
  float newP[15*15]; AtimesB(newP, I, st_.P, 15,15,15);
  memcpy(st_.P,newP,sizeof(newP));

  injectCorrection(dx);
}

void ESKF::updateYawFromVelocity(const Vec3& v, float sigma){
  if(sigma<=0) sigma = cfg_.sigma_yaw_cog;
  float speed = sqrtf(v.x*v.x+v.y*v.y);
  if(speed < 3.0f) return; // not reliable

  float yaw_meas = atan2f(v.y, v.x); // NED: atan2(E, N)
  // extract current yaw from q (R-> yaw)
  float R[9]; Rnb(st_.q,R);
  float yaw_est = atan2f(R[1], R[0]);
  float dyaw = yaw_meas - yaw_est;
  // wrap to [-pi,pi]
  while(dyaw >  M_PI) dyaw -= 2*M_PI;
  while(dyaw < -M_PI) dyaw += 2*M_PI;

  // 1D scalar update on δθ_z
  // Hθ = [0 0 1] on attitude error (others 0)
  // Build 15×1 H^T (only idx 6..8, use z= index 8)
  float Ht[15]={0}; Ht[6+2] = 1.0f; // δθ_z

  // S = H P H^T + R  => scalar
  float S = 0.0f;
  for(int i=0;i<15;i++) for(int j=0;j<15;j++) S += Ht[i]*st_.P[i*15+j]*Ht[j];
  S += sigma*sigma;
  if(S < 1e-6f) return;

  // K = P H^T S^{-1}
  float K[15]; memset(K,0,sizeof(K));
  for(int i=0;i<15;i++){
    float s=0; for(int j=0;j<15;j++) s += st_.P[i*15+j]*Ht[j];
    K[i] = s / S;
  }

  // dx = K * dyaw
  float dx[15]={0}; for(int i=0;i<15;i++) dx[i] = K[i]*dyaw;

  // P = (I-KH)P
  float I[15*15]; eye(I,15);
  for(int r=0;r<15;r++) for(int c=0;c<15;c++){
    I[r*15+c] -= K[r]*Ht[c];
  }
  float newP[15*15]; AtimesB(newP, I, st_.P, 15,15,15);
  memcpy(st_.P, newP, sizeof(newP));

  injectCorrection(dx);
}