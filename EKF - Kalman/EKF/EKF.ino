#include <Arduino.h>
#include <Wire.h>
#include "ESKF.h"   // uses global Vec3/Quat/make3()

// ---------------- GNSS types (yours) ----------------
enum class FIXTYPE : uint8_t { NOFIX=0, SPS=1, DGPS=2, RTKFIX=4, RTKFLOAT=5 };

struct GNSSFix {
  int   hour = 0;
  int   minute = 0;
  float second = 0;

  float lat = 0.0f;   // deg
  float lon = 0.0f;   // deg
  float alt = 0.0f;   // m

  float relNorth = 0.0f, relEast = 0.0f, relDown = 0.0f;
  float adjNorth = 0.0f, adjEast = 0.0f, adjDown = 0.0f;

  int   SIV  = 0;
  int   HDOP = 0;     // *100
  FIXTYPE type = FIXTYPE::NOFIX;
};

// ---------------- Pins / ports (ESP32 defaults) ----------------
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;

static constexpr int GNSS_RX = 16; // UM980 TX -> ESP32 RX
static constexpr int GNSS_TX = 17; // UM980 RX <- ESP32 TX
static constexpr uint32_t GNSS_BAUD = 115200;

HardwareSerial SerialGNSS(2);

// ---------------- Simple IMU wrapper (replace) ----------------
struct ImuSample { float gx, gy, gz; float ax, ay, az; };

class MyIMU {
public:
  bool begin(TwoWire& bus) {
    (void)bus;
    // TODO init your IMU
    return true;
  }
  ImuSample read() {
    ImuSample s{}; // TODO fill with SI units (rad/s, m/s^2)
    return s;
  }
};

MyIMU imu;

// ---------------- ESKF + origin ----------------
ESKF ekf;
ESKFConfig cfg;

bool   originSet = false;
double lat0_deg = 0.0, lon0_deg = 0.0;
float  h0_m = 0.0f;
float  lat0_rad = 0.0f;
float  m_per_deg_lat = 111320.0f, m_per_deg_lon = 111320.0f;

bool   havePrevGnss = false;
uint32_t prevGnssMs = 0;
Vec3   prevPn = make3();

// timing
uint32_t lastPrintMs = 0;
uint32_t lastImuMicros = 0;

// ------------- Utils -------------
Vec3 llaToNED(float lat_deg, float lon_deg, float alt_m) {
  float dLat_m = (lat_deg - (float)lat0_deg) * m_per_deg_lat;
  float dLon_m = (lon_deg - (float)lon0_deg) * (m_per_deg_lon * cosf(lat0_rad));
  float dAlt_m = (alt_m - h0_m);
  return make3(dLat_m, dLon_m, -dAlt_m); // NED: down positive
}

bool pollGNSS(GNSSFix& out) {
  (void)out;
  // TODO: your UM980 parser that fills 'out' and returns true on new fix
  return false;
}

float posSigmaForFix(const GNSSFix& f) {
  switch (f.type) {
    case FIXTYPE::RTKFIX:   return 0.03f;
    case FIXTYPE::RTKFLOAT: return 0.10f;
    default:                return 0.25f;
  }
}
float velSigmaForFix(const GNSSFix& f) {
  switch (f.type) {
    case FIXTYPE::RTKFIX:
    case FIXTYPE::RTKFLOAT: return 0.10f;
    default:                return 0.25f;
  }
}

// ---- helpers (put in a small header if you want) ----
static inline float vdot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3  vcross(const Vec3& a, const Vec3& b){
  return make3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline float vnorm(const Vec3& a){ return sqrtf(vdot(a,a)); }
static inline Vec3  vunit(const Vec3& a){ float n=vnorm(a); return (n>1e-6f)? make3(a.x/n,a.y/n,a.z/n) : make3(0,0,1); }

// Quaternion that rotates unit vector u -> v (both unit)
static inline Quat quatFromTwoUnitVectors(const Vec3& u, const Vec3& v){
  float d = vdot(u,v);
  if (d > 0.999999f) return {1,0,0,0};         // almost same
  if (d < -0.999999f){                          // opposite: choose any orthogonal axis
    Vec3 ortho = fabsf(u.x) < 0.9f ? make3(1,0,0) : make3(0,1,0);
    Vec3 axis = vunit(vcross(u, ortho));
    return {0, axis.x, axis.y, axis.z};        // 180° rotation
  }
  Vec3 axis = vcross(u,v);
  Quat q = { 1.0f + d, axis.x, axis.y, axis.z };
  ESKF::quatNormalize(q);
  return q;
}

// Rotate body vector to NED with Rnb(q)
static inline Vec3 rotateBodyToNED(const Quat& q, const Vec3& vb){
  float R[9]; ESKF::Rnb(q, R);
  return make3(
    R[0]*vb.x + R[1]*vb.y + R[2]*vb.z,
    R[3]*vb.x + R[4]*vb.y + R[5]*vb.z,
    R[6]*vb.x + R[7]*vb.y + R[8]*vb.z
  );
}

// ---- calibration result ----
struct CalibResult {
  Vec3 bg;     // gyro bias (rad/s)
  Vec3 ba;     // accel bias (m/s^2)
  Quat q0;     // initial attitude (body->NED) with yaw=0
  float g_est; // local gravity magnitude
};

// imu.read() must return SI: gyro rad/s, accel m/s^2
CalibResult calibrateStillAnyOrientation(MyIMU& imu, float seconds=6.0f, uint32_t rate_hz=400){
  const uint32_t N = (uint32_t)(seconds * rate_hz);
  const uint32_t us = 1000000UL / rate_hz;

  Vec3 sumG = make3(), sumA = make3();

  for (uint32_t k=0;k<N;k++){
    ImuSample s = imu.read();
    sumG.x += s.gx; sumG.y += s.gy; sumG.z += s.gz;
    sumA.x += s.ax; sumA.y += s.ay; sumA.z += s.az;
    delayMicroseconds(us);
  }

  Vec3 meanG = make3(sumG.x/N, sumG.y/N, sumG.z/N);
  Vec3 meanA = make3(sumA.x/N, sumA.y/N, sumA.z/N);

  // 1) Gyro bias
  Vec3 bg = meanG;

  // 2) Gravity magnitude and direction in body frame
  float g_est = vnorm(meanA);
  Vec3 a_hat  = (g_est > 1e-4f) ? make3(meanA.x/g_est, meanA.y/g_est, meanA.z/g_est) : make3(0,0,1);

  // 3) Initial attitude: rotate body a_hat to NED gravity dir [0,0,+1]
  Vec3 gdir_ned = make3(0,0,1);
  Quat q0 = quatFromTwoUnitVectors(a_hat, gdir_ned); // body->NED, yaw=0

  // 4) Accel bias: meanA = R^T * [0,0,+g_est] + ba  ->  ba = meanA - R^T*g
  // Compute R^T*g by rotating NED->[body] (inverse rotation): use q^{-1}
  Quat qi = { q0.w, -q0.x, -q0.y, -q0.z }; // unit inverse
  Vec3 g_ned = make3(0,0,g_est);
  // rotate NED vector into body frame with qi (NED->body):
  float Rnb[9]; ESKF::Rnb(qi, Rnb); // note: with qi this implements Rbn
  Vec3 g_body = make3(
    Rnb[0]*g_ned.x + Rnb[1]*g_ned.y + Rnb[2]*g_ned.z,
    Rnb[3]*g_ned.x + Rnb[4]*g_ned.y + Rnb[5]*g_ned.z,
    Rnb[6]*g_ned.x + Rnb[7]*g_ned.y + Rnb[8]*g_ned.z
  );
  Vec3 ba = make3(meanA.x - g_body.x, meanA.y - g_body.y, meanA.z - g_body.z);

  // (Optionally zero tiny components)
  if (fabsf(ba.x) < 1e-3f) ba.x = 0;
  if (fabsf(ba.y) < 1e-3f) ba.y = 0;
  if (fabsf(ba.z) < 1e-3f) ba.z = 0;

  return { bg, ba, q0, g_est };
}


// ------------- Setup -------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nEKF demo: IMU + GNSS (NED)");

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  if (!imu.begin(Wire)) Serial.println("IMU init failed!");

  SerialGNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);

  cfg.sigma_g   = 0.02f;
  cfg.sigma_a   = 0.25f;
  cfg.sigma_bg  = 0.0005f;
  cfg.sigma_ba  = 0.01f;
  cfg.sigma_gnss_pos = 0.20f;
  cfg.sigma_gnss_vel = 0.15f;
  cfg.sigma_yaw_cog  = 2.0f * (float)M_PI / 180.0f;

  ekf = ESKF(cfg);

CalibResult C = calibrateStillAnyOrientation(imu, 6.0f, 400);

// Save to NVS if you want (bg, ba, q0)
ekf.reset(
  /*p0*/ make3(0,0,0),
  /*v0*/ make3(0,0,0),
  /*q0*/ C.q0,           // roll/pitch from gravity, yaw=0
  /*bg*/ C.bg,
  /*ba*/ C.ba,
  /*Pdiag*/ 1.0f
);

  lastImuMicros = micros();
}

// ------------- Loop -------------
void loop() {
  // 1) IMU predict
  uint32_t nowUs = micros();
  float dt = (nowUs - lastImuMicros) * 1e-6f;
  if (dt <= 0) dt = 1e-4f;
  lastImuMicros = nowUs;

  ImuSample s = imu.read();
  ekf.predict(dt, make3(s.gx, s.gy, s.gz), make3(s.ax, s.ay, s.az));

  // 2) GNSS updates
  GNSSFix fix;
  if (pollGNSS(fix)) {
    if (!originSet && fix.type != FIXTYPE::NOFIX) {
      lat0_deg = fix.lat;
      lon0_deg = fix.lon;
      h0_m     = fix.alt;
      lat0_rad = (float)(lat0_deg * DEG_TO_RAD);
      ESKF::nedMetersPerDeg(lat0_rad, m_per_deg_lat, m_per_deg_lon);
      originSet = true;
      Serial.printf("Origin set: lat=%.7f lon=%.7f h=%.2f\n", lat0_deg, lon0_deg, h0_m);
    }

    if (originSet) {
      bool haveRelpos = (fabsf(fix.relNorth) > 0.001f || fabsf(fix.relEast) > 0.001f || fabsf(fix.relDown) > 0.001f);
      Vec3 p_ned;
      if (haveRelpos) {
        p_ned = make3(fix.relNorth + fix.adjNorth,
                      fix.relEast  + fix.adjEast,
                      fix.relDown  + fix.adjDown);
      } else {
        p_ned = llaToNED(fix.lat, fix.lon, fix.alt);
      }

      ekf.updatePositionNED(p_ned, posSigmaForFix(fix));

      uint32_t nowMs = millis();
      if (havePrevGnss) {
        float dtv = (nowMs - prevGnssMs) * 1e-3f;
        if (dtv > 0.02f && dtv < 0.5f) {
          Vec3 v_meas = make3(
            (p_ned.x - prevPn.x)/dtv,
            (p_ned.y - prevPn.y)/dtv,
            (p_ned.z - prevPn.z)/dtv
          );
          ekf.updateVelocityNED(v_meas, velSigmaForFix(fix));
          ekf.updateYawFromVelocity(v_meas, cfg.sigma_yaw_cog);
        }
      }
      prevPn = p_ned;
      prevGnssMs = nowMs;
      havePrevGnss = true;
    }
  }

  // 3) Debug @ ~10 Hz
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs > 100) {
    lastPrintMs = nowMs;
    const auto& st = ekf.state();

    // yaw from Rnb
    float R[9];
    {
      const float w=st.q.w,x=st.q.x,y=st.q.y,z=st.q.z;
      const float ww=w*w, xx=x*x, yy=y*y, zz=z*z;
      const float wx=w*x, wy=w*y, wz=w*z;
      const float xy=x*y, xz=x*z, yz=y*z;
      R[0]=ww+xx-yy-zz; R[1]=2*(xy-wz);   R[2]=2*(xz+wy);
      R[3]=2*(xy+wz);   R[4]=ww-xx+yy-zz;R[5]=2*(yz-wx);
      R[6]=2*(xz-wy);   R[7]=2*(yz+wx);  R[8]=ww-xx-yy+zz;
    }
    float yaw = atan2f(R[1], R[0]) * RAD_TO_DEG;

    Serial.printf("p[NED]=[%.2f %.2f %.2f] v=[%.2f %.2f %.2f] yaw=%.1f bg=[%.3f %.3f %.3f] ba=[%.2f %.2f %.2f]\n",
      st.p.x, st.p.y, st.p.z,
      st.v.x, st.v.y, st.v.z,
      yaw,
      st.bg.x, st.bg.y, st.bg.z,
      st.ba.x, st.ba.y, st.ba.z
    );
  }
}
