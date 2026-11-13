#pragma once
// Small, dependency-free helpers used by FlightCore (no Arduino headers on purpose,
// so the flight logic can be unit-tested on a PC).
#include <math.h>
#include <stdint.h>

// Exponential running mean / standard deviation.
struct RunStat {
  float mean = 0.0f, var = 0.0f;
  bool init = false;
  void reset() { mean = 0.0f; var = 0.0f; init = false; }
  void add(float x, float a = 0.02f) {
    if (!init) { mean = x; var = 0.0f; init = true; return; }
    float d = x - mean;
    mean += a * d;
    var = (1.0f - a) * (var + a * d * d);
  }
  float sd() const { return sqrtf(var > 0.0f ? var : 0.0f); }
};

inline float median3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

// Tracks the direction of "world up" expressed in the BODY frame by integrating
// the gyro (exact rotation, not small-angle Euler). Orientation-axis agnostic:
// the angle between "up now" and "up on the pad" is the tilt of the airframe,
// no matter how the board is mounted, and spin about the up axis is harmless.
class UpTracker {
 public:
  void set(float x, float y, float z) { _x = x; _y = y; _z = z; normalize(); }

  // Rotate the tracked vector by the body rotation w (deg/s) over dt seconds.
  void integrate(float gxDps, float gyDps, float gzDps, float dt) {
    const float D2R = 0.017453293f;
    float wx = gxDps * D2R, wy = gyDps * D2R, wz = gzDps * D2R;
    float w = sqrtf(wx * wx + wy * wy + wz * wz);
    float phi = w * dt;
    if (phi < 1e-6f) return;
    float nx = wx / w, ny = wy / w, nz = wz / w;
    float c = cosf(phi), s = sinf(phi);
    float ndotu = nx * _x + ny * _y + nz * _z;
    // frame rotates by +phi about n  =>  vector components rotate by -phi
    float cx = ny * _z - nz * _y;   // n x u
    float cy = nz * _x - nx * _z;
    float cz = nx * _y - ny * _x;
    float ux = _x * c - cx * s + nx * ndotu * (1.0f - c);
    float uy = _y * c - cy * s + ny * ndotu * (1.0f - c);
    float uz = _z * c - cz * s + nz * ndotu * (1.0f - c);
    _x = ux; _y = uy; _z = uz;
    normalize();
  }

  // Pull the estimate toward a measured gravity direction (ONLY valid when the
  // airframe is stationary - never call this in flight).
  void correct(float ax, float ay, float az, float gain) {
    float m = sqrtf(ax * ax + ay * ay + az * az);
    if (m < 1e-3f) return;
    _x += gain * (ax / m - _x);
    _y += gain * (ay / m - _y);
    _z += gain * (az / m - _z);
    normalize();
  }

  void setRef() { _rx = _x; _ry = _y; _rz = _z; }
  void setDisp() { _dx = _x; _dy = _y; _dz = _z; }          // reference used ONLY for the dashboard tilt readout
  float angleToDispDeg() const {
    float d = _x * _dx + _y * _dy + _z * _dz;
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return acosf(d) * 57.2957795f;
  }
  float ux() const { return _x; }
  float uy() const { return _y; }
  float uz() const { return _z; }

  float angleToRefDeg() const {
    float d = _x * _rx + _y * _ry + _z * _rz;
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return acosf(d) * 57.2957795f;
  }

 private:
  float _x = 0, _y = 0, _z = 1;
  float _rx = 0, _ry = 0, _rz = 1;
  float _dx = 0, _dy = 0, _dz = 1;
  void normalize() {
    float m = sqrtf(_x * _x + _y * _y + _z * _z);
    if (m < 1e-6f) { _x = 0; _y = 0; _z = 1; return; }
    _x /= m; _y /= m; _z /= m;
  }
};


// Full 3-D attitude as a quaternion, for the DASHBOARD only (the flight logic uses UpTracker).
//   * starts exactly at the measured pose (no guessed "up"), whatever way the board is mounted
//   * roll/pitch/yaw are SIGNED, right-hand rule about the chip's X/Y/Z axes, relative to a zero reference
//     (positive = counter-clockwise when looking from the +axis end towards the origin; for a board lying
//      flat with Z up, anticlockwise seen from above = positive yaw)
//   * roll/pitch are pulled back to gravity whenever the board is still (so they return to zero);
//     yaw can only come from the gyro (no magnetometer), so the caller freezes it at rest.
class Attitude {
 public:
  void initFromAccel(float ax, float ay, float az) {
    float m = sqrtf(ax * ax + ay * ay + az * az);
    if (m < 1e-3f) return;
    float ux = ax / m, uy = ay / m, uz = az / m;       // measured "up" in the body frame
    float ax_ = uy, ay_ = -ux;                         // axis = u x e_z
    float s = sqrtf(ax_ * ax_ + ay_ * ay_);
    if (s < 1e-6f) {
      if (uz > 0) { w = 1; x = y = z = 0; } else { w = 0; x = 1; y = z = 0; }
    } else {
      float ang = acosf(uz > 1 ? 1 : (uz < -1 ? -1 : uz));
      float sh = sinf(ang * 0.5f) / s;
      w = cosf(ang * 0.5f); x = ax_ * sh; y = ay_ * sh; z = 0;
    }
    setRef();
  }
  void integrate(float gxDps, float gyDps, float gzDps, float dt) {   // body-frame rates
    const float D2R = 0.017453293f;
    float wx = gxDps * D2R, wy = gyDps * D2R, wz = gzDps * D2R;
    float wm = sqrtf(wx * wx + wy * wy + wz * wz);
    float phi = wm * dt;
    if (phi < 1e-7f) return;
    float sh = sinf(phi * 0.5f) / wm;
    mulRight(cosf(phi * 0.5f), wx * sh, wy * sh, wz * sh);
  }
  void correct(float ax, float ay, float az, float gain) {            // tilt-only; never touches yaw
    float m = sqrtf(ax * ax + ay * ay + az * az);
    if (m < 1e-3f) return;
    float mx = ax / m, my = ay / m, mz = az / m;
    float ubx = 2 * (x * z - w * y), uby = 2 * (y * z + w * x), ubz = 1 - 2 * (x * x + y * y);   // predicted up, body frame
    float ex = my * ubz - mz * uby, ey = mz * ubx - mx * ubz, ez = mx * uby - my * ubx;        // m x u_b
    mulRight(1.0f, 0.5f * gain * ex, 0.5f * gain * ey, 0.5f * gain * ez);
  }
  void setRef() { rw = w; rx = x; ry = y; rz = z; }
  void euler(float &rollDeg, float &pitchDeg, float &yawDeg) const {   // of q_rel = conj(ref) * q
    float qw = rw * w + rx * x + ry * y + rz * z;
    float qx = rw * x - rx * w - ry * z + rz * y;
    float qy = rw * y + rx * z - ry * w - rz * x;
    float qz = rw * z - rx * y + ry * x - rz * w;
    const float R2D = 57.2957795f;
    rollDeg = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)) * R2D;
    float sp = 2 * (qw * qy - qz * qx);
    pitchDeg = asinf(sp > 1 ? 1 : (sp < -1 ? -1 : sp)) * R2D;
    yawDeg = atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) * R2D;
  }
 private:
  float w = 1, x = 0, y = 0, z = 0;
  float rw = 1, rx = 0, ry = 0, rz = 0;
  void mulRight(float bw, float bx, float by, float bz) {              // q = q * b, then normalise
    float nw = w * bw - x * bx - y * by - z * bz;
    float nx = w * bx + x * bw + y * bz - z * by;
    float ny = w * by - x * bz + y * bw + z * bx;
    float nz = w * bz + x * by - y * bx + z * bw;
    float m = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
    if (m < 1e-9f) return;
    w = nw / m; x = nx / m; y = ny / m; z = nz / m;
  }
};
