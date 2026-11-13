#pragma once
#include <Arduino.h>
#include <Wire.h>

/*
  Self-contained, failure-aware drivers for the MPU6500 and BMP280.

  Differences from the big controller's drivers, on purpose:
    - Every I2C transaction is checked. A failed read RETURNS FALSE and never leaves
      stale/uninitialised bytes that look like data.
    - "Stuck" detection: a sensor that keeps returning the identical bytes (frozen) or
      all-0x00 / all-0xFF (bus floating) is reported as failed.
    - Both can be re-initialised at runtime, WITHOUT long delays, so a sensor that
      browns out mid-flight can come back.
    - BMP280 compensation is done here (datasheet formula) so there is no library
      that could silently return garbage on a bad read.
*/

struct ImuSample {
  float ax, ay, az;      // g   (+-16 g range)
  float gx, gy, gz;      // deg/s (+-2000 dps), gyro bias already subtracted
  float tempC;
  uint32_t tUs;          // micros() at read
};

class Imu {
 public:
  bool begin(TwoWire &w, uint8_t addr);     // full init incl. device reset (~110 ms) - setup only
  bool quickInit();                         // re-apply config, no reset - safe to call in flight
  bool read(ImuSample &s);                  // false = no valid new data
  bool calibrateGyro(uint16_t samples, float &sdDps);  // keep still; false if it moved
  bool configIntact();                      // false if the chip silently reset (wrong range/sleep) -> call quickInit()
  bool present() const { return _present; }
  uint32_t errors() const { return _err; }
  float biasX() const { return _bx; } float biasY() const { return _by; } float biasZ() const { return _bz; }
 private:
  TwoWire *_w = nullptr; uint8_t _addr = 0x68;
  bool _present = false;
  float _bx = 0, _by = 0, _bz = 0;
  uint8_t _last[14] = {0}; uint16_t _same = 0;
  uint32_t _err = 0;
  bool wr(uint8_t reg, uint8_t val);
  bool rd(uint8_t reg, uint8_t *buf, uint8_t n);
  bool configure();
  bool readRaw(uint8_t *buf);
};

class Baro {
 public:
  bool begin(TwoWire &w, uint8_t addr);     // tries addr and addr^1 (0x76/0x77); accepts BMP280/BME280 ids
  bool reinit();                            // reconfigure after a dropout
  bool configIntact();                      // false if the chip reset itself -> call reinit()
  bool read(float &pressurePa, float &tempC);
  bool present() const { return _present; }
  uint32_t errors() const { return _err; }
 private:
  TwoWire *_w = nullptr; uint8_t _addr = 0x76;
  bool _present = false;
  uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
  uint8_t _last[6] = {0}; uint16_t _same = 0;
  uint32_t _err = 0;
  bool wr(uint8_t reg, uint8_t val);
  bool rd(uint8_t reg, uint8_t *buf, uint8_t n);
  bool configure();
  bool loadCalibration();
};
