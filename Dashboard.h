#pragma once
#include <Arduino.h>
#include "Config.h"

// ---- one consistent snapshot of everything the dashboard shows (filled by the flight task) ----
struct Snap {
  uint32_t ms;
  uint8_t state, reason, src;
  uint8_t baroOk, imuOk;
  uint8_t relayOn, fireCount, falseLaunches;
  uint8_t tiltValid, resumed, pinEnabled, pinState, gyroCalOk;
  uint16_t warn;
  float testRem, blindRem;                 // seconds, <0 = inactive
  float alt, peakAlt, climb, peakClimb, pressure, padPressure, baroTemp, imuTemp, vz;
  float ax, ay, az, mag, gx, gy, gz, tilt, maxG, maxGyro, maxTilt;
  float altNoiseCm, presNoisePa, accNoiseMg, gyroNoise, gyroBias, stillS;
  float imuHz, baroHz;
  uint32_t imuErr, baroErr, recov, i2cRecov;
  float flightT, apogeeT, ejectT;          // seconds, <0 = n/a
  Settings set;
};

// ---- small, fast-changing subset (IMU + altitude) updated every flight-loop iteration (~200 Hz) ----
struct FastSnap {
  uint32_t ms;
  float ax, ay, az, mag, gx, gy, gz, tilt, vz, alt, climb;
  float roll, pitch, yaw;                  // signed, right-hand rule about the chip's X / Y / Z (anticlockwise = +)
  uint8_t state, imuOk, baroOk;
};

enum CmdId : uint8_t {
  CMD_ARM = 1, CMD_DISARM, CMD_ZERO, CMD_GYROCAL, CMD_RESET, CMD_RESET_PEAKS,
  CMD_TEST, CMD_CANCEL, CMD_CLICK, CMD_BLIND, CMD_BLIND_CANCEL, CMD_SAVE, CMD_TILTZERO
};
struct Cmd { uint8_t id; int32_t a, b, c, d, e; };

// implemented in the .ino
bool readSnapshot(Snap &out);
bool readFast(FastSnap &out);
bool postCommand(const Cmd &c);

void dashboardBegin();       // start AP + HTTP server
void dashboardStop();        // radio fully off (flight)
void dashboardHandle();      // call from loop()
bool dashboardRunning();
