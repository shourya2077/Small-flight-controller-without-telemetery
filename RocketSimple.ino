/*
  RocketSimple - single-board parachute ejection controller
  ESP32-C3 Super Mini + BMP280 + MPU6500 + relay.   No radio telemetry; WiFi dashboard on the pad only.

  Goal: eject the parachute at apogee BY ANY MEANS, even if sensors die.

  Architecture
    flightTask (priority 10)   200 Hz: read sensors -> FlightCore -> relay. Owns I2C and the relay.
                               Never prints, never touches WiFi/flash, never blocks.
    loop() (priority 1)        WiFi access point + dashboard, flash saving of settings, serial log.
    RTC memory                 flight clock/peaks survive a watchdog / brown-out reset mid-flight.

  See README.md for wiring, the failure matrix, tuning and the pre-flight checklist.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "Config.h"
#include "FlightCore.h"
#include "Sensors.h"
#include "Ejector.h"
#include "RtcStore.h"
#include "Dashboard.h"

static FlightCore core;
static Imu imu;
static Baro baro;
static Ejector ej;
static Settings settings;
static Preferences prefs;

static QueueHandle_t cmdQ;
static portMUX_TYPE snapMux = portMUX_INITIALIZER_UNLOCKED;
static Snap g_snap;
static bool g_snapValid = false;
static FastSnap g_fast;
static bool g_fastValid = false;
static volatile bool g_wifiWanted = true;
static volatile bool g_settingsDirty = false;
static volatile bool g_padNow = true;
static volatile uint8_t g_stateMirror = 0;
static volatile uint8_t g_reasonMirror = 0;

// flight-task state ------------------------------------------------------------
static bool everImu = false, everBaro = false, gyroCalOk = false, resumed = false;
static uint32_t imuLastGood = 0, baroLastGood = 0, lastImuUs = 0;
static uint32_t lastImuTry = 0, lastBaroTry = 0, lastBaroPoll = 0, lastHealth = 0, lastBusRec = 0;
static uint32_t recov = 0, busRec = 0;
static float baroTempC = 0, imuTempC = 0;
static uint32_t imuCnt = 0, baroCnt = 0, rateT0 = 0;
static float imuHz = 0, baroHz = 0;
static bool testActive = false; static uint32_t testEnd = 0;
static uint32_t lastRtcMs = 0; static uint8_t lastSavedState = 255;
static uint32_t lastSnapMs = 0;
static bool pinArmed = false; static uint32_t pinLowSince = 0, pinHighSince = 0;

// ============================================================================ helpers
static uint16_t clampU(long v, long lo, long hi) { return (uint16_t)(v < lo ? lo : (v > hi ? hi : v)); }

static void sanitize(Settings &s) {
  s.apogeeTimerS = clampU(s.apogeeTimerS, 3, 120);
  s.maxTimerS    = clampU(s.maxTimerS, 5, 180);
  if (s.maxTimerS <= s.apogeeTimerS) s.maxTimerS = s.apogeeTimerS + 1;
  s.tiltDeg      = clampU(s.tiltDeg, 60, 170);
  s.pulseMs      = clampU(s.pulseMs, 300, 5000);
  s.launchGx10   = (uint8_t)clampU(s.launchGx10, 15, 120);
}

static void applySettings() {
  core.cfg.apogeeTimerS = settings.apogeeTimerS;
  core.cfg.maxTimerS    = settings.maxTimerS;
  core.cfg.tiltDeg      = settings.tiltDeg;
  core.cfg.launchGx10   = settings.launchGx10;
}

static void loadSettings() {
  Settings d;
  if (prefs.begin("rfc", true)) {
    d.apogeeTimerS = prefs.getUShort("ap", d.apogeeTimerS);
    d.maxTimerS    = prefs.getUShort("mx", d.maxTimerS);
    d.tiltDeg      = prefs.getUShort("td", d.tiltDeg);
    d.pulseMs      = prefs.getUShort("pm", d.pulseMs);
    d.launchGx10   = prefs.getUChar("lg", d.launchGx10);
    prefs.end();
  }
  sanitize(d);
  settings = d;
}

static void saveSettings() {
  Settings s;
  portENTER_CRITICAL(&snapMux); s = settings; portEXIT_CRITICAL(&snapMux);
  if (prefs.begin("rfc", false)) {
    prefs.putUShort("ap", s.apogeeTimerS); prefs.putUShort("mx", s.maxTimerS);
    prefs.putUShort("td", s.tiltDeg);      prefs.putUShort("pm", s.pulseMs);
    prefs.putUChar("lg", s.launchGx10);
    prefs.end();
  }
}

// ---- I2C bus: free a slave that is holding SDA low (classic after a brown-out / reset) ----
static void i2cFreeBus() {
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  digitalWrite(PIN_I2C_SCL, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);       // generate a STOP
  digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
}
static void i2cStart() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}
static void i2cRecover() {
  Wire.end();
  i2cFreeBus();
  i2cStart();
  busRec++;
}

static bool launchPinTripped(uint32_t now) {
  if (PIN_LAUNCH_SENSE < 0) return false;
  bool high = digitalRead(PIN_LAUNCH_SENSE) == (LAUNCH_SENSE_TRIP_LEVEL_HIGH ? HIGH : LOW);
  if (!pinArmed) {                                   // only trust the pin after it was seen in the "on pad" state for 1.5 s
    if (!high) { if (!pinLowSince) pinLowSince = now | 1; if ((uint32_t)(now - pinLowSince) >= 1500) pinArmed = true; }
    else pinLowSince = 0;
    return false;
  }
  if (high) { if (!pinHighSince) pinHighSince = now | 1; return (uint32_t)(now - pinHighSince) >= LAUNCH_SENSE_DEBOUNCE_MS; }
  pinHighSince = 0;
  return false;
}

// ============================================================================ commands
static void handleCmd(const Cmd &c, uint32_t now) {
  switch (c.id) {
    case CMD_ARM:    core.arm(); break;
    case CMD_DISARM: core.disarm(); break;
    case CMD_ZERO:   core.zeroBaro(now); break;
    case CMD_GYROCAL:
      if (core.isPad() && imu.present()) {
        float sd;
        gyroCalOk = imu.calibrateGyro(300, sd);
        core.gbx = core.gby = core.gbz = 0;
        imuLastGood = millis();
      }
      break;
    case CMD_RESET:
      if (core.isPad() || core.state == FState::EJECTED) {
        testActive = false;
        core.resetAll(now);
        core.zeroBaro(now);                   // landing site is not the pad: take a fresh reference
        rtcClear(); resumed = false; lastSavedState = 255;
      }
      break;
    case CMD_RESET_PEAKS: core.resetPeaks(); break;
    case CMD_TILTZERO:    if (core.isPad()) core.zeroTilt(); break;
    case CMD_TEST:
      if (core.isPad() && !testActive && !ej.busy()) { testActive = true; testEnd = now + TEST_COUNTDOWN_MS; }
      break;
    case CMD_CANCEL: testActive = false; break;
    case CMD_CLICK:
      if (core.isPad() && !testActive) ej.click(now, CLICK_PULSE_MS);
      break;
    case CMD_BLIND:        core.startBlind(now, (uint32_t)(c.a < 0 ? 0 : c.a)); break;
    case CMD_BLIND_CANCEL: core.cancelBlind(); break;
    case CMD_SAVE:
      if (core.isPad()) {
        Settings s;
        s.apogeeTimerS = clampU(c.a, 3, 120);
        s.maxTimerS    = clampU(c.b, 5, 180);
        s.tiltDeg      = clampU(c.c, 60, 170);
        s.launchGx10   = (uint8_t)clampU(c.d, 15, 120);
        s.pulseMs      = clampU(c.e, 300, 5000);
        sanitize(s);
        portENTER_CRITICAL(&snapMux); settings = s; portEXIT_CRITICAL(&snapMux);
        applySettings();
        g_settingsDirty = true;
      }
      break;
  }
}

bool postCommand(const Cmd &c) { return cmdQ && xQueueSend(cmdQ, &c, 0) == pdTRUE; }

bool readFast(FastSnap &out) {
  portENTER_CRITICAL(&snapMux);
  bool ok = g_fastValid;
  if (ok) out = g_fast;
  portEXIT_CRITICAL(&snapMux);
  return ok;
}

static float displayTilt() { return (core.inFlight() || core.state == FState::EJECTED) ? core.tiltDeg : core.tiltDispDeg; }

static inline float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

// Updated EVERY loop iteration (200 Hz). The IMU numbers are lightly smoothed (DISPLAY_TAU_MS) FOR DISPLAY ONLY -
// the flight logic never sees this filter.
static void updateFast(uint32_t now, bool imuOk, bool baroOk) {
  static float sm[11]; static bool smInit = false; static uint32_t last = 0;
  float in[11] = { core.ax, core.ay, core.az, core.mag, core.gx, core.gy, core.gz,
                   displayTilt(), core.vz, 0, 0 };
  float dtMs = smInit ? (float)(uint32_t)(now - last) : 0.0f;
  last = now;
  float a = (DISPLAY_TAU_MS <= 0 || !smInit) ? 1.0f : dtMs / ((float)DISPLAY_TAU_MS + dtMs);
  if (a > 1.0f) a = 1.0f;
  for (int k = 0; k < 9; k++) sm[k] += a * (in[k] - sm[k]);
  sm[9]  = wrap180(sm[9]  + a * wrap180(core.rollDeg  - sm[9]));      // roll / pitch / yaw smoothed through the wrap
  sm[10] = wrap180(sm[10] + a * wrap180(core.yawDeg   - sm[10]));
  static float spitch = 0;
  spitch += a * (core.pitchDeg - spitch);
  if (!smInit) { for (int k = 0; k < 9; k++) sm[k] = in[k]; sm[9] = core.rollDeg; sm[10] = core.yawDeg; spitch = core.pitchDeg; smInit = true; }

  FastSnap f;
  f.ms = now; f.ax = sm[0]; f.ay = sm[1]; f.az = sm[2]; f.mag = sm[3];
  f.gx = sm[4]; f.gy = sm[5]; f.gz = sm[6]; f.tilt = sm[7]; f.vz = sm[8];
  f.roll = sm[9]; f.pitch = spitch; f.yaw = sm[10];
  f.alt = core.alt; f.climb = core.climb;
  f.state = (uint8_t)core.state; f.imuOk = imuOk; f.baroOk = baroOk;
  portENTER_CRITICAL(&snapMux);
  g_fast = f; g_fastValid = true;
  portEXIT_CRITICAL(&snapMux);
}

bool readSnapshot(Snap &out) {
  portENTER_CRITICAL(&snapMux);
  bool ok = g_snapValid;
  if (ok) out = g_snap;
  portEXIT_CRITICAL(&snapMux);
  return ok;
}

// ============================================================================ snapshot
static void buildSnapshot(uint32_t now, bool imuOk, bool baroOk) {
  Snap s;
  memset((void *)&s, 0, sizeof(s));
  s.ms = now;
  s.state = (uint8_t)core.state; s.reason = (uint8_t)core.reason; s.src = (uint8_t)core.launchSrc;
  s.baroOk = baroOk; s.imuOk = imuOk;
  s.relayOn = ej.isOn(); s.fireCount = ej.fireCount(); s.falseLaunches = core.falseLaunches;
  s.tiltValid = core.tiltValid; s.resumed = resumed;
  s.pinEnabled = (PIN_LAUNCH_SENSE >= 0) && pinArmed; s.pinState = pinHighSince != 0; s.gyroCalOk = gyroCalOk;

  uint16_t w = 0;
  float still = core.stillSeconds(now);
  if (!baroOk) w |= 1;
  if (!imuOk)  w |= 2;
  if (core.isPad() && imuOk && still < 1.0f) w |= 4;
  if (core.altStat.init && core.altStat.sd() > 1.0f) w |= 8;
  if (imuOk && still > 2.0f && fabsf(core.gRef - 1.0f) > 0.20f) w |= 16;   // wrong range/scale (offsets up to ~0.1 g are normal)
  if (!gyroCalOk && imu.present()) w |= 32;
  if (core.state == FState::PAD_DISARMED) w |= 64;
  if (core.isPad() && core.baroSettling(now)) w |= 128;
  if (core.falseLaunches) w |= 256;
  if (resumed) w |= 512;
  if (!core.tiltValid) w |= 1024;
  s.warn = w;

  s.testRem = testActive ? (float)(int32_t)(testEnd - now) * 0.001f : -1.0f;
  s.blindRem = core.blindRemainingS(now);

  s.alt = core.alt; s.peakAlt = core.peakAlt; s.climb = core.climb; s.peakClimb = core.peakClimb;
  s.pressure = core.pressure; s.padPressure = core.padP; s.baroTemp = baroTempC; s.imuTemp = imuTempC; s.vz = core.vz;
  s.ax = core.ax; s.ay = core.ay; s.az = core.az; s.mag = core.mag;
  s.gx = core.gx; s.gy = core.gy; s.gz = core.gz;
  s.tilt = displayTilt(); s.maxG = core.maxAccelG; s.maxGyro = core.maxGyroDps; s.maxTilt = core.maxTiltDeg;

  s.altNoiseCm = core.altStat.sd() * 100.0f;
  s.presNoisePa = core.presStat.sd();
  s.accNoiseMg = core.magStat.sd() * 1000.0f;
  s.gyroNoise = fmaxf(core.gxStat.sd(), fmaxf(core.gyStat.sd(), core.gzStat.sd()));
  s.gyroBias = fmaxf(fabsf(imu.biasX() + core.gbx), fmaxf(fabsf(imu.biasY() + core.gby), fabsf(imu.biasZ() + core.gbz)));
  s.stillS = still;
  s.imuHz = imuHz; s.baroHz = baroHz;
  s.imuErr = imu.errors(); s.baroErr = baro.errors(); s.recov = recov; s.i2cRecov = busRec;

  bool fl = core.state == FState::FLIGHT || core.state == FState::LAUNCH_CAND;
  s.flightT = fl ? (float)(uint32_t)(now - core.t0) * 0.001f : -1.0f;
  s.apogeeT = ((fl || core.state == FState::EJECTED) && core.peakAlt > 0.5f) ? core.apogeeAtMs * 0.001f : -1.0f;
  s.ejectT = core.state == FState::EJECTED ? (float)(uint32_t)(core.ejectAtMs - core.t0) * 0.001f : -1.0f;
  portENTER_CRITICAL(&snapMux);
  s.set = settings;
  g_snap = s; g_snapValid = true;
  portEXIT_CRITICAL(&snapMux);
}

// ============================================================================ flight task
// 5 ms loop. Guard: if the RTOS tick were ever slower than 1 kHz, pdMS_TO_TICKS(5) would be 0 and the loop would never yield.
#define FLIGHT_PERIOD_TICKS ((pdMS_TO_TICKS(5) > 0) ? pdMS_TO_TICKS(5) : (TickType_t)1)

static void flightTask(void *) {
  esp_task_wdt_add(NULL);
  TickType_t wake = xTaskGetTickCount();
  rateT0 = millis();

  for (;;) {
    uint32_t now = millis();

    // ---- 1. commands from the dashboard
    Cmd cm;
    while (xQueueReceive(cmdQ, &cm, 0) == pdTRUE) handleCmd(cm, now);

    // ---- 2. IMU (200 Hz). A failing sensor is only retried every 50 ms so a dead bus can't eat the loop.
    bool imuOkPrev = everImu && (uint32_t)(now - imuLastGood) < 100;
    if (imuOkPrev || (uint32_t)(now - lastImuTry) >= 50) {
      lastImuTry = now;
      ImuSample s;
      if (imu.read(s)) {
        float dt = lastImuUs ? (float)(uint32_t)(s.tUs - lastImuUs) * 1e-6f : 0.005f;
        lastImuUs = s.tUs;
        imuLastGood = now; everImu = true; imuTempC = s.tempC; imuCnt++;
        core.feedImu(now, dt, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
      }
    }

    // ---- 3. barometer (40 Hz)
    bool baroOkPrev = everBaro && (uint32_t)(now - baroLastGood) < 250;
    if ((uint32_t)(now - lastBaroPoll) >= 25 && (baroOkPrev || (uint32_t)(now - lastBaroTry) >= 50)) {
      lastBaroPoll = now; lastBaroTry = now;
      float p, t;
      if (baro.read(p, t)) {
        baroLastGood = now; everBaro = true; baroTempC = t; baroCnt++;
        core.feedBaro(now, p);
      }
    }

    // ---- 4. health
    bool imuOk  = everImu  && (uint32_t)(now - imuLastGood)  < 100;
    bool baroOk = everBaro && (uint32_t)(now - baroLastGood) < 250;

    updateFast(now, imuOk, baroOk);

    // ---- 5. recovery (every 500 ms): re-init a sensor that dropped or silently reset; free a stuck bus
    if ((uint32_t)(now - lastHealth) >= 500) {
      lastHealth = now;
      if (!imuOk)                       { if (imu.quickInit()) recov++; }
      else if (!imu.configIntact())     { imu.quickInit(); recov++; }
      if (!baroOk)                      { if (baro.reinit()) recov++; }
      else if (!baro.configIntact())    { baro.reinit(); recov++; }
      if (!imuOk && !baroOk && (uint32_t)(now - lastBusRec) >= 1000) { lastBusRec = now; i2cRecover(); }
      // 1-second rate counters
      float dtS = (now - rateT0) * 0.001f;
      if (dtS >= 1.0f) { imuHz = imuCnt / dtS; baroHz = baroCnt / dtS; imuCnt = baroCnt = 0; rateT0 = now; }
    }

    // ---- 6. the decision
    core.tick(now, imuOk, baroOk, launchPinTripped(now));

    // ---- 7. fire
    if (core.takeFire()) {
      testActive = false;
      if (ej.busy()) ej.abort();            // a bench click/test must never block the real thing
      ej.fire(now, settings.pulseMs, RELAY_REPEAT_PULSE);
    }
    if (testActive) {
      if (!core.isPad()) testActive = false;
      else if ((int32_t)(now - testEnd) >= 0) { testActive = false; ej.fire(now, settings.pulseMs, RELAY_REPEAT_PULSE); }
    }
    ej.update(now);

    // ---- 8. RTC heartbeat (flight + 2 min after ejection)
    bool keep = core.state == FState::FLIGHT ||
                (core.state == FState::EJECTED && (uint32_t)(now - core.ejectAtMs) < 120000UL);
    if (keep && ((uint32_t)(now - lastRtcMs) >= 100 || lastSavedState != (uint8_t)core.state)) {
      rtcSave(core, now); lastRtcMs = now; lastSavedState = (uint8_t)core.state;
    }

    // ---- 9. policy flags for loop()
    g_padNow = core.isPad();
    g_stateMirror = (uint8_t)core.state; g_reasonMirror = (uint8_t)core.reason;
    bool wifiOff = WIFI_OFF_IN_FLIGHT &&
                   (core.state == FState::FLIGHT ||
                    (core.state == FState::EJECTED && (uint32_t)(now - core.ejectAtMs) < WIFI_BACK_AFTER_EJECT_MS));
    g_wifiWanted = !wifiOff;

    // ---- 10. LED
    if (PIN_LED >= 0) {
      bool on = false; uint32_t ph = now % 2000;
      switch (core.state) {
        case FState::PAD_ARMED:    on = (imuOk && baroOk) ? (ph < 80) : ((now / 100) & 1); break;
        case FState::PAD_DISARMED: on = (ph < 1000); break;
        case FState::EJECTED:      on = (now % 1000) < 500; break;
        default: on = false;
      }
      digitalWrite(PIN_LED, LED_ACTIVE_LOW ? !on : on);
    }

    // ---- 11. snapshot for the dashboard (50 Hz)
    if ((uint32_t)(now - lastSnapMs) >= 20) { lastSnapMs = now; buildSnapshot(now, imuOk, baroOk); }

    esp_task_wdt_reset();
    vTaskDelayUntil(&wake, FLIGHT_PERIOD_TICKS);
  }
}

// ============================================================================ setup / loop
void setup() {
  // The relay pin is the very first thing configured.
  ej.begin(PIN_RELAY, RELAY_ACTIVE_HIGH);
  if (PIN_LED >= 0) { pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LED_ACTIVE_LOW ? HIGH : LOW); }
  if (PIN_LAUNCH_SENSE >= 0) pinMode(PIN_LAUNCH_SENSE, INPUT_PULLUP);

  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);               // never block waiting for a USB host
#endif
  delay(200);
  Serial.println("\n=== RocketSimple booting ===");

  loadSettings();
  applySettings();

  FlightSave sv;
  bool resume = rtcLoad(sv);

  i2cFreeBus();
  i2cStart();
  bool baroFound = baro.begin(Wire, BMP_ADDR);
  bool imuFound  = imu.begin(Wire, MPU_ADDR);
  Serial.printf("[BMP280] %s\n[MPU6500] %s\n", baroFound ? "OK" : "NOT FOUND", imuFound ? "OK" : "NOT FOUND");

  uint32_t now = millis();
  if (resume) {
    // Reset in mid-flight: carry on, no calibration, no waiting.
    sv.elapsedMs += REBOOT_PENALTY_MS;
    if (sv.state == (uint8_t)FState::EJECTED) sv.ejectAgeMs += REBOOT_PENALTY_MS;
    core.restore(now, sv);
    resumed = true; gyroCalOk = true;
    Serial.printf("!! RESUMED FLIGHT after reset: state=%u elapsed=%lu ms\n", sv.state, (unsigned long)sv.elapsedMs);
  } else {
    rtcClear();
    if (imuFound) {
      float sd = 0;
      for (int tries = 0; tries < 3 && !gyroCalOk; tries++) {
        gyroCalOk = imu.calibrateGyro(300, sd);
        if (!gyroCalOk) delay(300);
      }
      Serial.printf("[MPU6500] gyro cal %s (sd %.2f dps)\n", gyroCalOk ? "OK" : "FAILED - keep still & press Re-cal gyro", sd);
    }
    // pad pressure = median of ~24 samples
    float pv[24]; int pn = 0; float tmp;
    if (baroFound) {
      for (int i = 0; i < 24; i++) {
        float p;
        if (baro.read(p, tmp)) pv[pn++] = p;
        delay(40);
      }
    }
    core.resetAll(millis());
    if (pn >= 10) {
      for (int i = 1; i < pn; i++) { float k = pv[i]; int j = i - 1; while (j >= 0 && pv[j] > k) { pv[j + 1] = pv[j]; j--; } pv[j + 1] = k; }
      core.seedPadPressure(pv[pn / 2]);
      Serial.printf("[BMP280] pad pressure %.2f hPa\n", pv[pn / 2] / 100.0f);
    }
  }

  g_wifiWanted = !(resume && sv.state == (uint8_t)FState::FLIGHT);
  cmdQ = xQueueCreate(8, sizeof(Cmd));
  imuLastGood = baroLastGood = millis();
  xTaskCreate(flightTask, "flight", 8192, NULL, 10, NULL);
  Serial.println("Flight task running. ARMED.");
}

void loop() {
  static uint8_t lastState = 255;
  bool want = g_wifiWanted;
  if (want && !dashboardRunning()) { dashboardBegin(); Serial.printf("WiFi AP '%s' up -> http://192.168.4.1\n", WIFI_SSID); }
  if (!want && dashboardRunning()) { dashboardStop(); Serial.println("WiFi off (flight)"); }
  dashboardHandle();

  if (g_settingsDirty && g_padNow) { g_settingsDirty = false; saveSettings(); }

  uint8_t st = g_stateMirror;
  if (st != lastState) {
    lastState = st;
    Serial.printf("[state] %u reason=%u\n", st, g_reasonMirror);
  }
  delay(1);
}
