#include "FlightCore.h"

void FlightCore::resetPeaks() {
  peakAlt = 0; peakClimb = 0; apogeeAtMs = 0;
  maxAccelG = 0; maxGyroDps = 0; maxTiltDeg = 0;
}

void FlightCore::resetAll(uint32_t now) {
  state = FState::PAD_ARMED;
  reason = Reason::NONE;
  launchSrc = LSrc::NONE;
  t0 = 0; ejectAtMs = 0; fireRequest = false; falseLaunches = 0;
  tiltValid = true;
  dispLatched = false;
  baroInit = false;                 // next baro sample re-seeds the filter
  alt = 0; rawAlt = 0; climb = 0;
  spikeCount = 0; baroLaunchStreak = 0; baroDropStreak = 0;
  baroSettleUntil = now + BARO_SETTLE_MS;
  accelActive = false; haveModerate = false;
  boostSeen = boostEnded = beActive = false;
  tiltActive = false; lowActive = false; vzActive = false; vz = 0;
  highAccelMs = 0; blindEndMs = 0;
  resetPeaks();
}

void FlightCore::arm()    { if (state == FState::PAD_DISARMED) { state = FState::PAD_ARMED; accelActive = false; baroLaunchStreak = 0; } }
void FlightCore::disarm() { if (state == FState::PAD_ARMED)   { state = FState::PAD_DISARMED; accelActive = false; blindEndMs = 0; } }

void FlightCore::zeroBaro(uint32_t now) {
  if (!isPad()) return;
  padPInit = false;                 // adopt the next pressure sample as the new pad reference
  baroInit = false;
  alt = 0; climb = 0; baroLaunchStreak = 0;
  baroSettleUntil = now + BARO_SETTLE_MS;
  altStat.reset(); presStat.reset();
}

void FlightCore::startBlind(uint32_t now, uint32_t sec) {
  if (state != FState::PAD_ARMED) return;
  if (sec < 5) sec = 5;
  if (sec > 3600) sec = 3600;
  blindEndMs = now + sec * 1000UL;
  if (blindEndMs == 0) blindEndMs = 1;
}

// ------------------------------------------------------------------ barometer
void FlightCore::feedBaro(uint32_t now, float p) {
  pressure = p;
  if (!padPInit) { padP = p; padPInit = true; }

  float raw = 44330.0f * (1.0f - powf(p / padP, 0.190294957f));
  bool gap = (uint32_t)(now - lastBaroMs) > 1000;

  // Reject a lone absurd jump (bus glitch / bad compensation). Three in a row are accepted
  // as real so a genuine fast change can never lock the filter out.
  if (baroInit && !gap && fabsf(raw - h0) > BARO_SPIKE_M && spikeCount < 3) {
    spikeCount++;
    return;
  }
  spikeCount = 0;

  float m;
  if (!baroInit || gap) { h0 = h1 = h2 = raw; m = raw; }
  else { h2 = h1; h1 = h0; h0 = raw; m = median3(h0, h1, h2); }
  rawAlt = raw;

  if (!baroInit || gap) {
    alt = m; climb = 0; baroInit = true;
  } else {
    float dt = (uint32_t)(now - lastBaroMs) * 0.001f;
    if (dt < 0.005f) dt = 0.005f;
    float prev = alt;
    alt += 0.35f * (m - alt);
    float inst = (alt - prev) / dt;
    climb += 0.15f * (inst - climb);
  }
  lastBaroMs = now;

  if (isPad()) {
    if (fabsf(climb) < 1.0f && fabsf(alt) < 25.0f) {
      padP += 0.002f * (p - padP);          // follow slow weather drift (tau ~12 s) - frozen during a launch
      altStat.add(raw, 0.02f);
      presStat.add(p, 0.02f);
    }
    if (state == FState::PAD_ARMED && !baroSettling(now) &&
        alt > BARO_LAUNCH_ALT_M && climb > BARO_LAUNCH_CLIMB) {
      if (baroLaunchStreak < 255) baroLaunchStreak++;
    } else {
      baroLaunchStreak = 0;
    }
  } else if (inFlight()) {
    if (alt > peakAlt) { peakAlt = alt; apogeeAtMs = now - t0; }
    if (climb > peakClimb) peakClimb = climb;
    vz += VZ_BARO_BLEND * (climb - vz);          // keeps the IMU integral from drifting while the baro lives
    if (vz > vzMax) vzMax = vz;
    if (state == FState::FLIGHT && (uint32_t)(now - t0) >= MIN_ASCENT_MS && peakAlt > BARO_MIN_PEAK_M &&
        (peakAlt - alt) > BARO_DROP_M && climb < -0.5f) {
      if (baroDropStreak < 255) baroDropStreak++;
    } else {
      baroDropStreak = 0;
    }
  }
}

// ------------------------------------------------------------------------ IMU
void FlightCore::feedImu(uint32_t now, float dt, float ax_, float ay_, float az_,
                         float gx_, float gy_, float gz_) {
  ax = ax_; ay = ay_; az = az_;
  mag = sqrtf(ax * ax + ay * ay + az * az);
  float cx = gx_ - gbx, cy = gy_ - gby, cz = gz_ - gbz;
  gx = cx; gy = cy; gz = cz;
  gyroMag = sqrtf(cx * cx + cy * cy + cz * cz);

  if (!magSlowInit) { magSlow = mag; magSlowInit = true; }
  float magDev = fabsf(mag - magSlow);          // how much |a| just moved relative to its own recent average
  magSlow += 0.01f * (mag - magSlow);
  if (!gsInit) { gsx = cx; gsy = cy; gsz = cz; gsInit = true; }
  float dvx = fabsf(cx - gsx), dvy = fabsf(cy - gsy), dvz = fabsf(cz - gsz);
  gsx += 0.01f * (cx - gsx); gsy += 0.01f * (cy - gsy); gsz += 0.01f * (cz - gsz);

  // Start EXACTLY at the measured pose (not at a guessed "up"), so everything reads zero at power-on.
  if (!upInit && mag > 0.5f) { up.set(ax, ay, az); up.setRef(); up.setDisp(); upInit = true; }
  if (!attInit && mag > 0.5f) { att.initFromAccel(ax, ay, az); attInit = true; }

  if (dt > 0.1f) {                       // we lost rotation data: tilt can no longer be trusted in flight
    if (inFlight()) tiltValid = false;
    dt = 0.02f;
  }
  if (dt <= 0.0f) dt = 0.005f;
  if (tiltValid || isPad()) up.integrate(cx, cy, cz, dt);
  // display attitude: at rest on the pad the gyro is ignored below a small dead-band so yaw cannot creep
  if (attInit && !(isPad() && gyroMag < GYRO_DEADBAND_DPS)) att.integrate(cx, cy, cz, dt);

  if (isPad()) {
    // "Still" = |a| steady AND gyro steady. Deliberately NOT "|a| == 1.00 g" and NOT "gyro == 0": a stock
    // MPU6500 has up to ~80 mg accel offset and several dps of gyro offset, which would otherwise make the
    // board look permanently in motion (reference never latches, gyro bias never learned).
    bool still = mag > 0.7f && mag < 1.3f && magDev < 0.04f &&
                 dvx < 0.8f && dvy < 0.8f && dvz < 0.8f &&
                 fabsf(cx) < 12.0f && fabsf(cy) < 12.0f && fabsf(cz) < 12.0f;
    if (still) {
      if (!stillActive) { stillActive = true; stillSince = now; }
      // gyro-bias trim: fast while the offset is large, gentle once it is small
      gbx += (fabsf(cx) > 0.4f ? 0.02f : 0.003f) * cx;
      gby += (fabsf(cy) > 0.4f ? 0.02f : 0.003f) * cy;
      gbz += (fabsf(cz) > 0.4f ? 0.02f : 0.003f) * cz;
      up.correct(ax, ay, az, 0.02f);                                  // gravity correction: ONLY when still
      if (attInit) att.correct(ax, ay, az, 0.02f);
      if (!gRefInit) { gRef = mag; gRefInit = true; } else gRef += 0.01f * (mag - gRef);
      if (gyroMag < 0.6f) { if (!quietActive) { quietActive = true; quietSince = now; } } else quietActive = false;
      if ((uint32_t)(now - stillSince) >= 1000) up.setRef();          // pad-vertical reference (flight logic)
      // dashboard zero: once the gyro has settled (bias learned) - so readings start at, and return to, 0
      if (!dispLatched && quietActive && (uint32_t)(now - quietSince) >= 1000) {
        up.setDisp(); if (attInit) att.setRef(); dispLatched = true;
      }
      magStat.add(mag); gxStat.add(cx); gyStat.add(cy); gzStat.add(cz);
    } else {
      stillActive = false; quietActive = false;
    }
    if (!tiltValid) tiltValid = true;
  }
  tiltDeg = up.angleToRefDeg();
  tiltDispDeg = up.angleToDispDeg();
  if (attInit) att.euler(rollDeg, pitchDeg, yawDeg);

  // Vertical speed: specific force along world-up minus the rest reading (gRef, so offset/scale error at
  // 1 g cancels). Runs from the first moment of a possible launch (accelActive) so no burn time is lost.
  if (inFlight() || (state == FState::PAD_ARMED && accelActive)) {
    if (tiltValid) {
      float fup = ax * up.ux() + ay * up.uy() + az * up.uz();
      vz += (fup - gRef) * 9.80665f * dt;
      if (vz > vzMax) vzMax = vz;
    }
  } else if (isPad()) {
    vz = 0;
  }

  if (state != FState::EJECTED) {
    if (mag > maxAccelG) maxAccelG = mag;
    if (gyroMag > maxGyroDps) maxGyroDps = gyroMag;
  }
  if (inFlight()) {
    if (tiltValid && tiltDeg > maxTiltDeg) maxTiltDeg = tiltDeg;
    if (mag > launchG()) highAccelMs += dt * 1000.0f;
  }
}

// ----------------------------------------------------------------- state machine
void FlightCore::beginLaunch(uint32_t now, uint32_t tStart, LSrc src, bool confirmed) {
  t0 = tStart;
  launchSrc = src;
  state = confirmed ? FState::FLIGHT : FState::LAUNCH_CAND;
  peakAlt = (baroInit && alt > 0) ? alt : 0;
  peakClimb = 0; apogeeAtMs = 0; maxTiltDeg = 0;
  boostSeen = boostEnded = beActive = false;
  tiltActive = false; lowActive = false; baroDropStreak = 0;
  vzMax = 0;
  if (src == LSrc::BARO) vz = climb;
  highAccelMs = (src == LSrc::ACCEL) ? (float)(uint32_t)(now - tStart) : 0.0f;
  blindEndMs = 0; accelActive = false; baroLaunchStreak = 0;
}

void FlightCore::eject(uint32_t now, Reason r) {
  state = FState::EJECTED;
  reason = r;
  ejectAtMs = now;
  fireRequest = true;
}

// How far the IMU-only vertical speed can be wrong. With a live baro it is continuously pulled back to the
// baro (small). Without one, accelerometer scale error grows with peak speed and offset error with time.
float FlightCore::vzSigma(uint32_t now, bool baroOk) const {
  if (baroOk) return VZ_SIGMA_MIN;
  uint32_t ref = (lastBaroOkMs > t0) ? lastBaroOkMs : t0;
  float s = VZ_SIGMA_MIN + VZ_SIGMA_SCALE * vzMax + VZ_SIGMA_DRIFT * ((uint32_t)(now - ref) * 0.001f);
  return s > VZ_SIGMA_MAX ? VZ_SIGMA_MAX : s;
}

void FlightCore::checkApogee(uint32_t now, bool imuOk, bool baroOk) {
  uint32_t el = (uint32_t)(now - t0);
  uint32_t maxMs = (uint32_t)cfg.maxTimerS * 1000UL;
  uint32_t apMs  = (uint32_t)cfg.apogeeTimerS * 1000UL;

  // "Are we still going up?"  Baro is trusted first; the IMU speed estimate only when there is no baro.
  bool baroClimbing = baroOk && climb > CLIMB_BLOCK_MPS;
  float sig = vzSigma(now, baroOk);
  bool imuClimbing  = !baroOk && tiltValid && vz > (VZ_BLOCK_MPS - sig);   // true speed could be up to 'sig' above the estimate
  bool climbing = baroClimbing || imuClimbing;

  // E: absolute last resort. Never held off by anything.
  if (el >= maxMs) { eject(now, Reason::MAXTIMER); return; }

  // D: backup timer. Needs no sensor. Only held off if a WORKING sensor says we are still climbing fast
  //    (timer set too short for this flight); E above bounds the hold-off.
  if (el >= apMs) {
    bool strongClimb = (baroOk && climb > TIMER_DEFER_CLIMB) ||
                       (!baroOk && imuOk && tiltValid && vz > TIMER_DEFER_VZ);
    if (!strongClimb) { eject(now, Reason::TIMER); return; }
  }

  // burnout tracking (used by the free-fall detector)
  if (imuOk) {
    if (mag > launchG()) boostSeen = true;
    if (boostSeen && !boostEnded) {
      if (mag < BOOST_END_G) {
        if (!beActive) { beActive = true; beSince = now; }
        if ((uint32_t)(now - beSince) >= 100) boostEnded = true;
      } else beActive = false;
    }
  }

  if (el < MIN_ASCENT_MS) {              // rail wobble / thrust transients: no sensor apogee yet
    tiltActive = false; lowActive = false; vzActive = false; baroDropStreak = 0;
    return;
  }

  // A: barometer (descending below the peak - cannot fire while climbing by construction)
  if (baroOk) {
    if (baroDropStreak >= BARO_DROP_STREAK ||
        (baroDropStreak >= 2 && (peakAlt - alt) > 12.0f)) { eject(now, Reason::BARO); return; }
  }

  // B: tilt (gyro-integrated). Only counts once we are no longer climbing: a cartwheeling rocket that is
  //    still rising must not get a chute at speed.
  if (imuOk && tiltValid && tiltDeg > (float)cfg.tiltDeg && !climbing) {
    if (!tiltActive) { tiltActive = true; tiltSinceMs = now; }
    if ((uint32_t)(now - tiltSinceMs) >= TILT_SUSTAIN_MS) { eject(now, Reason::TILT); return; }
  } else tiltActive = false;

  // C: IMU vertical speed has turned negative (works with a dead barometer)
  if (imuOk && tiltValid && vz < (VZ_APOGEE_MPS - sig) && !baroClimbing) {
    if (!vzActive) { vzActive = true; vzSince = now; }
    if ((uint32_t)(now - vzSince) >= VZ_SUSTAIN_MS) { eject(now, Reason::VELOCITY); return; }
  } else vzActive = false;

  // C2: free-fall (sensed accel ~0 g), only once nothing says we are still climbing.
  if (imuOk && boostEnded && mag < FREEFALL_G && !climbing) {
    if (!lowActive) { lowActive = true; lowSince = now; }
    if ((uint32_t)(now - lowSince) >= FREEFALL_SUSTAIN_MS) { eject(now, Reason::FREEFALL); return; }
  } else lowActive = false;
}

void FlightCore::tick(uint32_t now, bool imuOk, bool baroOk, bool pinTripped) {
  if (baroOk) lastBaroOkMs = now;
  if (imuOk && mag > BARO_LAUNCH_CROSSCHECK_G) { haveModerate = true; lastModerateMs = now; }

  switch (state) {
    case FState::PAD_DISARMED:
    case FState::EJECTED:
      break;

    case FState::PAD_ARMED: {
      if (blindEndMs && (int32_t)(now - blindEndMs) >= 0) {
        beginLaunch(now, now, LSrc::BLIND, true);
        break;
      }
      if (pinTripped) { beginLaunch(now, now, LSrc::PIN, true); break; }

      // 1) acceleration
      if (imuOk && mag > launchG()) {
        if (!accelActive) { accelActive = true; accelSince = now; }
        if ((uint32_t)(now - accelSince) >= LAUNCH_ACCEL_SUSTAIN_MS) {
          beginLaunch(now, accelSince, LSrc::ACCEL, false);
          break;
        }
      } else accelActive = false;

      // 2) barometer (must be corroborated by the IMU whenever the IMU is alive)
      if (baroOk && baroLaunchStreak >= BARO_LAUNCH_STREAK) {
        bool corroborated = !imuOk || (haveModerate && (uint32_t)(now - lastModerateMs) < 1500);
        if (corroborated) beginLaunch(now, now, LSrc::BARO, true);
      }
      break;
    }

    case FState::LAUNCH_CAND: {
      bool confirmed = (highAccelMs >= LAUNCH_CONFIRM_HIGH_MS) || (baroOk && alt >= LAUNCH_CONFIRM_ALT_M);
      if (confirmed) { state = FState::FLIGHT; }
      else if ((uint32_t)(now - t0) > LAUNCH_CONFIRM_WINDOW_MS) {   // it was only a bump
        state = FState::PAD_ARMED;
        if (falseLaunches < 255) falseLaunches++;
        accelActive = false; baroLaunchStreak = 0;
      }
      break;
    }

    case FState::FLIGHT:
      checkApogee(now, imuOk, baroOk);
      break;
  }
}

// ------------------------------------------------------------------- persistence
void FlightCore::exportSave(uint32_t now, FlightSave& s) const {
  s.state = (uint8_t)state; s.reason = (uint8_t)reason; s.src = (uint8_t)launchSrc;
  s.elapsedMs = (state == FState::FLIGHT || state == FState::EJECTED) ? (uint32_t)(now - t0) : 0;
  s.ejectAgeMs = (state == FState::EJECTED) ? (uint32_t)(now - ejectAtMs) : 0;
  s.apogeeAtMs = apogeeAtMs;
  s.padP = padP; s.peakAlt = peakAlt; s.peakClimb = peakClimb;
  s.maxG = maxAccelG; s.maxGyro = maxGyroDps; s.maxTilt = maxTiltDeg;
}

void FlightCore::restore(uint32_t now, const FlightSave& s) {
  resetAll(now);
  padP = s.padP; padPInit = true;
  state = (FState)s.state; reason = (Reason)s.reason; launchSrc = (LSrc)s.src;
  t0 = now - s.elapsedMs;
  apogeeAtMs = s.apogeeAtMs; peakAlt = s.peakAlt; peakClimb = s.peakClimb;
  maxAccelG = s.maxG; maxGyroDps = s.maxGyro; maxTiltDeg = s.maxTilt;
  tiltValid = false;                        // orientation reference was lost with the reset
  boostSeen = true; boostEnded = s.elapsedMs > 4000;
  baroSettleUntil = now;
  if (state == FState::EJECTED) {
    ejectAtMs = now - s.ejectAgeMs;
    fireRequest = s.ejectAgeMs < 6000;      // reset hit during/just after the pulse: run the pulse again
  }
}
