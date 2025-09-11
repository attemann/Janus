#pragma once
#include <Arduino.h>
#include <math.h>

struct Vec3 { float x,y,z; };
struct Quat { float w,x,y,z; };

inline Vec3 make3(float x=0,float y=0,float z=0){ return {x,y,z}; }
inline Quat qIdentity(){ return {1,0,0,0}; }

struct ESKFConfig {
  float g = 9.80665f;
  // Continuous-time noise densities (tune to your IMU):
  float sigma_g   = 0.02f;   // rad/s/√Hz  (gyro noise)
  float sigma_a   = 0.25f;   // m/s^2/√Hz  (accel noise)
  float sigma_bg  = 0.0005f; // rad/s/√Hz  (gyro bias RW)
  float sigma_ba  = 0.01f;   // m/s^2/√Hz  (accel bias RW)

  // GNSS measurement stddevs (tune based on fix type):
  float sigma_gnss_pos = 0.20f;  // m
  float sigma_gnss_vel = 0.15f;  // m/s
  float sigma_yaw_cog  = 2.0f * (M_PI/180.0f); // rad
};

struct ESKFState {
  Vec3 p;    // NED m
  Vec3 v;    // NED m/s
  Quat q;    // body->NED
  Vec3 bg;   // rad/s
  Vec3 ba;   // m/s^2
  float P[15*15]; // covariance (row-major)
};


class ESKF {
public:
  explicit ESKF(const ESKFConfig& cfg=ESKFConfig());
  void reset(const Vec3& p0, const Vec3& v0, const Quat& q0, const Vec3& bg0, const Vec3& ba0, float Pdiag=1.0f);

  // Prediction with raw IMU (body frame):
  void predict(float dt, const Vec3& gyro, const Vec3& acc);

  // GNSS updates:
  void updatePositionNED(const Vec3& p_meas, float sigma= -1.0f);
  void updateVelocityNED(const Vec3& v_meas, float sigma= -1.0f);

  // Yaw from course-over-ground (when speed is high):
  void updateYawFromVelocity(const Vec3& v_meas_ned, float sigma_rad = -1.0f);

  const ESKFState& state() const { return st_; }

  // Helpers:
  static Quat fromEuler(float roll, float pitch, float yaw);
  static void quatNormalize(Quat& q);
  static Vec3 nedGravity(float g){ return make3(0,0,g); } // +z down in NED
  static void nedMetersPerDeg(float lat_rad, float& m_per_deg_lat, float& m_per_deg_lon);
    static void Rnb(const Quat& q, float R[9]);  // 3x3

private:
  ESKFConfig cfg_;
  ESKFState st_;

  // math helpers
  static Vec3 add(const Vec3&a,const Vec3&b);
  static Vec3 sub(const Vec3&a,const Vec3&b);
  static Vec3 scale(const Vec3&a,float s);
  static Vec3 cross3(const Vec3&a,const Vec3&b);
  static Quat qMul(const Quat& a, const Quat& b);
  static Vec3 rotateBodyToNED(const Quat& q, const Vec3& vb);
  static void skew(const Vec3& v, float S[9]); // 3x3


  // covariance ops
  void propagateCov(float dt, const Vec3& omega_b, const Vec3& f_b, const float Rnb_[9]);
  void injectCorrection(const float dx[15]);

  // generic small-matrix helpers
  static void eye(float* A, int n);
  static void addInPlace(float* A, const float* B, int n);
  static void AtimesB(float* C, const float* A, const float* B, int n, int m, int p);
  static void AtimesBT(float* C, const float* A, const float* B, int n, int m, int p);
  static void AplusAtimesBtimesAt(float* P, const float* F, const float* Q, float dt, int n);
  static bool solveSymmetric3(const float S[9], const float z[3], float K[9]); // small 3x3 solve for K=PH^T S^{-1}
  static bool invert3(const float A[9], float inv[9]);
};
