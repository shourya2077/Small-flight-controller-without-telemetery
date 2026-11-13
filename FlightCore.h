#pragma once
/*
  FlightCore - the whole "when do we fire the parachute" brain.

  Deliberately has NO Arduino dependency: it is fed samples + health flags and
  answers with a state and a one-shot fire request. That lets it be unit-tested
  on a PC (see sim/sim_test.cpp) against synthetic flights and sensor failures.

  States
    PAD_ARMED     on the pad, watching for launch (default after boot)
    PAD_DISARMED  bench mode, nothing can fire automatically
    LAUNCH_CAND   acceleration spike seen; must be CONFIRMED within 2 s or it is
                  discarded as a bump (protects against a false eject while handling)
    FLIGHT        launch confirmed; every apogee detector + timers are live
    EJECTED       fire request issued (latched until reset)

  Launch detection (any one starts the flight):
    1. accel  > launch threshold for 100 ms (confirmed by 250 ms of it OR +5 m of altitude)
    2. baro   +6 m and > 4 m/s climb (needs IMU to show > 1.5 g recently, if IMU is healthy)
    3. optional launch-sense pin
    4. dashboard "blind timer"  (for the no-sensor case)

  Apogee / eject detection (ANY one fires, they are independent, not a vote):
    A. barometer: altitude fell >3 m below the peak (and is descending) for ~150 ms
    B. tilt:     gyro-integrated tilt from pad-vertical above threshold for 400 ms
    C. IMU velocity: integrated vertical speed (accel along tracked 'up', blended with baro) < -3 m/s for 150 ms
    C2. free-fall: sensed accel < 0.3 g for 200 ms after burnout, and not still climbing (baro AND IMU speed agree)
    D. backup timer  (from launch)               - needs NO sensor at all
    E. max timer     (from launch, last resort)  - needs NO sensor at all
*/
#include <stdint.h>
#include <math.h>
#include "Filters.h"

#ifndef FC_FREEFALL_G
#define FC_FREEFALL_G 0.30f   // (only overridden by the PC test to isolate the velocity detector)
#endif

enum class FState : uint8_t { PAD_ARMED = 0, PAD_DISARMED = 1, LAUNCH_CAND = 2, FLIGHT = 3, EJECTED = 4 };
enum class Reason : uint8_t { NONE = 0, BARO = 1, TILT = 2, FREEFALL = 3, TIMER = 4, MAXTIMER = 5, VELOCITY = 6 };
enum class LSrc   : uint8_t { NONE = 0, ACCEL = 1, BARO = 2, PIN = 3, BLIND = 4 };

struct CoreCfg {
  uint16_t apogeeTimerS = 12;
  uint16_t maxTimerS    = 20;
  uint16_t tiltDeg      = 110;
  uint8_t  launchGx10   = 30;
};

// What is kept in RTC memory so a mid-flight reset can pick the flight back up.
struct FlightSave {
  uint8_t  state, reason, src;
  uint32_t elapsedMs, ejectAgeMs, apogeeAtMs;
  float    padP, peakAlt, peakClimb, maxG, maxGyro, maxTilt;
};

class FlightCore {
 public:
  // ---- configuration ----
  CoreCfg cfg;

  // ---- state ----
  FState state = FState::PAD_ARMED;
  Reason reason = Reason::NONE;
  LSrc   launchSrc = LSrc::NONE;
  uint32_t t0 = 0;            // millis() of launch start
  uint32_t ejectAtMs = 0;
  uint8_t  falseLaunches = 0;
  bool     tiltValid = true;

  // ---- barometer ----
  bool  baroInit = false;
  float padP = 101325.0f;     // pad reference pressure (Pa)
  bool  padPInit = false;
  float pressure = 0, rawAlt = 0, alt = 0, climb = 0;
  float peakAlt = 0, peakClimb = 0;
  uint32_t apogeeAtMs = 0;    // ms after launch at which peakAlt was reached

  // ---- IMU (bias-corrected) ----
  float ax = 0, ay = 0, az = 1, mag = 1;
  float gx = 0, gy = 0, gz = 0, gyroMag = 0;
  float tiltDeg = 0, maxTiltDeg = 0, maxAccelG = 0, maxGyroDps = 0;
  float vz = 0;                      // vertical speed estimate (m/s): IMU-integrated, pulled toward baro when available
  float rollDeg = 0, pitchDeg = 0, yawDeg = 0;   // SIGNED display attitude (dashboard only), relative to the zero reference
  float tiltDispDeg = 0;             // dashboard tilt (since power-on / 'zero tilt'); the flight logic uses tiltDeg
  float gRef = 1.0f;                 // measured |a| at rest on the pad (absorbs accel offset/scale error)
  float gbx = 0, gby = 0, gbz = 0;   // residual gyro bias trimmed on the pad

  // ---- accuracy statistics, measured while sitting still on the pad ----
  RunStat altStat, presStat, magStat, gxStat, gyStat, gzStat;

  // ---- API ----
  void resetAll(uint32_t now);                 // boot / "reset for next flight"
  void seedPadPressure(float pa) { padP = pa; padPInit = true; }
  void resetPeaks();
  void zeroTilt() { dispLatched = false; }
  void arm();
  void disarm();
  void zeroBaro(uint32_t now);
  void startBlind(uint32_t now, uint32_t sec);
  void cancelBlind() { blindEndMs = 0; }
  float blindRemainingS(uint32_t now) const {
    return blindEndMs ? (float)(int32_t)(blindEndMs - now) * 0.001f : -1.0f;
  }

  void feedBaro(uint32_t now, float pressurePa);
  void feedImu(uint32_t now, float dt, float ax, float ay, float az, float gx, float gy, float gz);
  void tick(uint32_t now, bool imuOk, bool baroOk, bool pinTripped);

  bool takeFire() { bool f = fireRequest; fireRequest = false; return f; }
  bool isPad() const { return state == FState::PAD_ARMED || state == FState::PAD_DISARMED; }
  bool inFlight() const { return state == FState::LAUNCH_CAND || state == FState::FLIGHT; }
  float stillSeconds(uint32_t now) const { return stillActive ? (now - stillSince) * 0.001f : 0.0f; }
  bool baroSettling(uint32_t now) const { return (int32_t)(now - baroSettleUntil) < 0; }
  float launchG() const { return cfg.launchGx10 * 0.1f; }

  void exportSave(uint32_t now, FlightSave& s) const;
  void restore(uint32_t now, const FlightSave& s);

 private:
  bool fireRequest = false;
  UpTracker up;

  // baro pipeline
  float h0 = 0, h1 = 0, h2 = 0;   // last raw altitudes (median-of-3)
  uint32_t lastBaroMs = 0;
  uint8_t spikeCount = 0;
  uint8_t baroLaunchStreak = 0;
  uint8_t baroDropStreak = 0;
  uint32_t baroSettleUntil = 0;

  // imu pipeline
  bool stillActive = false; uint32_t stillSince = 0;
  float highAccelMs = 0;
  bool accelActive = false; uint32_t accelSince = 0;
  bool haveModerate = false; uint32_t lastModerateMs = 0;
  bool boostSeen = false, boostEnded = false, beActive = false; uint32_t beSince = 0;
  bool tiltActive = false; uint32_t tiltSinceMs = 0;
  bool lowActive = false;  uint32_t lowSince = 0;
  bool vzActive = false;   uint32_t vzSince = 0;
  uint32_t lastBaroOkMs = 0; float vzMax = 0;
  float vzSigma(uint32_t now, bool baroOk) const;
  float magSlow = 1.0f; bool magSlowInit = false, gRefInit = false, dispLatched = false;
  Attitude att; bool attInit = false, upInit = false;
  float gsx = 0, gsy = 0, gsz = 0; bool gsInit = false;     // slow gyro average (still detection that tolerates a gyro offset)
  bool quietActive = false; uint32_t quietSince = 0;
  uint32_t blindEndMs = 0;

  void beginLaunch(uint32_t now, uint32_t tStart, LSrc src, bool confirmed);
  void eject(uint32_t now, Reason r);
  void checkApogee(uint32_t now, bool imuOk, bool baroOk);

  // ---------------- tuning constants ----------------
  static constexpr float    GYRO_DEADBAND_DPS = 0.5f;    // display attitude ignores the gyro below this while resting on the pad
  static constexpr uint32_t LAUNCH_ACCEL_SUSTAIN_MS = 100;
  static constexpr uint32_t LAUNCH_CONFIRM_WINDOW_MS = 2000;
  static constexpr float    LAUNCH_CONFIRM_HIGH_MS = 250.0f;
  static constexpr float    LAUNCH_CONFIRM_ALT_M = 5.0f;
  static constexpr float    BARO_LAUNCH_ALT_M = 6.0f;
  static constexpr float    BARO_LAUNCH_CLIMB = 4.0f;
  static constexpr uint8_t  BARO_LAUNCH_STREAK = 3;
  static constexpr float    BARO_LAUNCH_CROSSCHECK_G = 1.5f;
  static constexpr uint32_t BARO_SETTLE_MS = 3000;
  static constexpr float    BARO_SPIKE_M = 50.0f;
  static constexpr uint32_t MIN_ASCENT_MS = 1000;      // detectors A-C ignored this long after launch
  static constexpr float    BARO_DROP_M = 3.0f;
  static constexpr uint8_t  BARO_DROP_STREAK = 6;
  static constexpr float    BARO_MIN_PEAK_M = 5.0f;
  static constexpr uint32_t TILT_SUSTAIN_MS = 400;
  static constexpr float    BOOST_END_G = 1.5f;
  static constexpr float    FREEFALL_G = FC_FREEFALL_G;
  static constexpr uint32_t FREEFALL_SUSTAIN_MS = 200;
  static constexpr float    CLIMB_BLOCK_MPS = 3.0f;      // IMU-only detectors (B, C, C2) are held off while baro says we climb faster than this
  static constexpr float    VZ_BLOCK_MPS = 3.0f;         //   ...or, with no baro, while the IMU speed estimate (+ its uncertainty) says so
  static constexpr float    VZ_SIGMA_MIN = 1.0f;         // IMU-speed uncertainty model (m/s): min, 3% of peak speed, 0.8 m/s per second without baro
  static constexpr float    VZ_SIGMA_SCALE = 0.04f;
  static constexpr float    VZ_SIGMA_DRIFT = 0.8f;
  static constexpr float    VZ_SIGMA_MAX = 40.0f;
  static constexpr float    TIMER_DEFER_CLIMB = 3.0f;    // backup timer D is held off while a sensor says we are still climbing (E never is)
  static constexpr float    TIMER_DEFER_VZ = 5.0f;
  static constexpr float    VZ_APOGEE_MPS = -2.0f;      // plus the uncertainty: fires at vz < -(2 + sigma)
  static constexpr uint32_t VZ_SUSTAIN_MS = 150;
  static constexpr float    VZ_BARO_BLEND = 0.03f;
};
