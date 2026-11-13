#include "Sensors.h"

// ============================================================================ IMU
static constexpr uint8_t R_SMPLRT = 0x19, R_CONFIG = 0x1A, R_GYRO = 0x1B, R_ACC = 0x1C,
                         R_ACC2 = 0x1D, R_DATA = 0x3B, R_PWR = 0x6B, R_WHO = 0x75;
static constexpr float ACC_LSB_PER_G = 2048.0f;   // +-16 g
static constexpr float GYRO_LSB_PER_DPS = 16.4f;  // +-2000 dps

bool Imu::wr(uint8_t reg, uint8_t val) {
  _w->beginTransmission(_addr);
  _w->write(reg); _w->write(val);
  return _w->endTransmission() == 0;
}

bool Imu::rd(uint8_t reg, uint8_t *buf, uint8_t n) {
  _w->beginTransmission(_addr);
  _w->write(reg);
  if (_w->endTransmission(false) != 0) return false;
  if (_w->requestFrom((int)_addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!_w->available()) return false;
    buf[i] = _w->read();
  }
  return true;
}

bool Imu::configure() {
  uint8_t who = 0;
  if (!rd(R_WHO, &who, 1)) return false;
  if (who != 0x70 && who != 0x71 && who != 0x68 && who != 0x73) return false;   // 6500 / 9250 / 6050 / 9255
  bool ok = wr(R_PWR, 0x01);          // wake, PLL clock
  ok &= wr(R_CONFIG, 0x02);           // gyro DLPF ~92 Hz (3.9 ms delay)
  ok &= wr(R_SMPLRT, 0x00);
  ok &= wr(R_GYRO, 0x18);             // +-2000 dps: a tumbling airframe must never clip the gyro
  ok &= wr(R_ACC, 0x18);              // +-16 g: motor thrust must never clip the accelerometer
  ok &= wr(R_ACC2, 0x02);             // accel DLPF ~99 Hz (2.9 ms delay)
  uint8_t chk = 0;
  ok &= rd(R_ACC, &chk, 1);
  return ok && chk == 0x18;
}

bool Imu::begin(TwoWire &w, uint8_t addr) {
  _w = &w; _addr = addr; _same = 0;
  uint8_t who = 0;
  if (!rd(R_WHO, &who, 1)) { _present = false; return false; }
  wr(R_PWR, 0x80);                    // device reset
  delay(100);
  _present = configure();
  delay(10);
  return _present;
}

bool Imu::quickInit() {
  if (!_w) return false;
  _same = 0;
  _present = configure();
  return _present;
}

bool Imu::configIntact() {
  if (!_w) return false;
  uint8_t a = 0, g = 0, p = 0;
  if (!rd(R_ACC, &a, 1) || !rd(R_GYRO, &g, 1) || !rd(R_PWR, &p, 1)) return false;
  return a == 0x18 && g == 0x18 && (p & 0x40) == 0;      // ranges as configured, not in sleep
}

bool Imu::readRaw(uint8_t *buf) {
  if (!rd(R_DATA, buf, 14)) { _err++; return false; }
  bool allSame = true, all00 = true, allFF = true;
  for (uint8_t i = 0; i < 14; i++) {
    if (buf[i] != _last[i]) allSame = false;
    if (buf[i] != 0x00) all00 = false;
    if (buf[i] != 0xFF) allFF = false;
  }
  if (all00 || allFF) { _err++; return false; }               // floating / dead bus
  if (allSame) { if (_same < 65535) _same++; } else _same = 0;
  memcpy(_last, buf, 14);
  if (_same >= 40) { _err++; return false; }                   // frozen output
  return true;
}

bool Imu::read(ImuSample &s) {
  if (!_present && !_w) return false;
  uint8_t b[14];
  if (!readRaw(b)) return false;
  auto i16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  s.ax = i16(0) / ACC_LSB_PER_G;
  s.ay = i16(2) / ACC_LSB_PER_G;
  s.az = i16(4) / ACC_LSB_PER_G;
  s.tempC = i16(6) / 333.87f + 21.0f;
  s.gx = i16(8)  / GYRO_LSB_PER_DPS - _bx;
  s.gy = i16(10) / GYRO_LSB_PER_DPS - _by;
  s.gz = i16(12) / GYRO_LSB_PER_DPS - _bz;
  s.tUs = micros();
  _present = true;
  return true;
}

bool Imu::calibrateGyro(uint16_t n, float &sdDps) {
  _bx = _by = _bz = 0;
  double sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < n; i++) {
    ImuSample s;
    if (read(s)) {
      sx += s.gx; sy += s.gy; sz += s.gz;
      qx += (double)s.gx * s.gx; qy += (double)s.gy * s.gy; qz += (double)s.gz * s.gz;
      got++;
    }
    delay(3);
  }
  sdDps = 99.0f;
  if (got < n * 8 / 10 || got < 20) return false;
  double mx = sx / got, my = sy / got, mz = sz / got;
  double vx = qx / got - mx * mx, vy = qy / got - my * my, vz = qz / got - mz * mz;
  float sd = sqrtf((float)fmax(vx, fmax(vy, vz)));
  sdDps = sd;
  if (sd > 1.0f) return false;                     // it moved while calibrating
  if (fabs(mx) > 30 || fabs(my) > 30 || fabs(mz) > 30) return false;   // implausible bias
  _bx = (float)mx; _by = (float)my; _bz = (float)mz;
  return true;
}

// ============================================================================ BARO
bool Baro::wr(uint8_t reg, uint8_t val) {
  _w->beginTransmission(_addr);
  _w->write(reg); _w->write(val);
  return _w->endTransmission() == 0;
}

bool Baro::rd(uint8_t reg, uint8_t *buf, uint8_t n) {
  _w->beginTransmission(_addr);
  _w->write(reg);
  if (_w->endTransmission(false) != 0) return false;
  if (_w->requestFrom((int)_addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!_w->available()) return false;
    buf[i] = _w->read();
  }
  return true;
}

bool Baro::loadCalibration() {
  uint8_t c[24];
  if (!rd(0x88, c, 24)) return false;
  auto u16 = [&](int i) { return (uint16_t)(c[i] | (c[i + 1] << 8)); };
  auto s16 = [&](int i) { return (int16_t)(c[i] | (c[i + 1] << 8)); };
  T1 = u16(0); T2 = s16(2); T3 = s16(4);
  P1 = u16(6); P2 = s16(8); P3 = s16(10); P4 = s16(12); P5 = s16(14);
  P6 = s16(16); P7 = s16(18); P8 = s16(20); P9 = s16(22);
  return T1 != 0 && T1 != 0xFFFF && P1 != 0 && P1 != 0xFFFF;
}

bool Baro::configure() {
  uint8_t id = 0;
  if (!rd(0xD0, &id, 1)) return false;
  if (id != 0x56 && id != 0x57 && id != 0x58 && id != 0x60) return false;   // BMP280 / BME280
  if (!loadCalibration()) return false;
  // standby 0.5 ms, IIR x4, pressure x8, temperature x1, normal mode  (~40 Hz, ~0.2 m noise)
  bool ok = wr(0xF5, 0x08);
  ok &= wr(0xF4, 0x33);
  return ok;
}

bool Baro::begin(TwoWire &w, uint8_t addr) {
  _w = &w;
  for (uint8_t k = 0; k < 2; k++) {
    _addr = k == 0 ? addr : (addr ^ 1);
    uint8_t id = 0;
    if (!rd(0xD0, &id, 1)) continue;
    wr(0xE0, 0xB6);                     // soft reset
    delay(10);
    _same = 0;
    _present = configure();
    if (_present) return true;
  }
  _present = false;
  return false;
}

bool Baro::configIntact() {
  if (!_w) return false;
  uint8_t m = 0, f = 0;
  if (!rd(0xF4, &m, 1) || !rd(0xF5, &f, 1)) return false;
  return m == 0x33 && (f & 0x1C) == 0x08;
}

bool Baro::reinit() {
  if (!_w) return false;
  _same = 0;
  uint8_t first = _addr;
  for (uint8_t k = 0; k < 2; k++) {          // also finds the chip if it only shows up at the other address
    _addr = k == 0 ? first : (first ^ 1);
    if (configure()) { _present = true; return true; }
  }
  _addr = first;
  _present = false;
  return false;
}

bool Baro::read(float &pa, float &tc) {
  if (!_w) return false;
  uint8_t b[6];
  if (!rd(0xF7, b, 6)) { _err++; return false; }

  bool same = true;
  for (uint8_t i = 0; i < 6; i++) if (b[i] != _last[i]) { same = false; break; }
  if (same) { if (_same < 65535) _same++; } else _same = 0;
  memcpy(_last, b, 6);
  if (_same >= 60) { _err++; return false; }                           // frozen output

  int32_t adcP = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  int32_t adcT = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
  if (adcP == 0x80000 || adcT == 0x80000 || adcP == 0 || adcT == 0 ||
      adcP == 0xFFFFF || adcT == 0xFFFFF) { _err++; return false; }    // skipped / bus-stuck values

  double v1 = (((double)adcT) / 16384.0 - ((double)T1) / 1024.0) * (double)T2;
  double v2 = ((((double)adcT) / 131072.0 - ((double)T1) / 8192.0) *
               (((double)adcT) / 131072.0 - ((double)T1) / 8192.0)) * (double)T3;
  double tFine = v1 + v2;
  double T = tFine / 5120.0;

  v1 = tFine / 2.0 - 64000.0;
  v2 = v1 * v1 * (double)P6 / 32768.0;
  v2 = v2 + v1 * (double)P5 * 2.0;
  v2 = v2 / 4.0 + (double)P4 * 65536.0;
  v1 = ((double)P3 * v1 * v1 / 524288.0 + (double)P2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * (double)P1;
  if (v1 == 0.0) { _err++; return false; }
  double p = 1048576.0 - (double)adcP;
  p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = (double)P9 * p * p / 2147483648.0;
  v2 = p * (double)P8 / 32768.0;
  p = p + (v1 + v2 + (double)P7) / 16.0;

  if (!(p > 30000.0 && p < 110000.0) || !(T > -50.0 && T < 120.0)) { _err++; return false; }
  pa = (float)p; tc = (float)T;
  return true;
}
