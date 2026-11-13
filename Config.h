#pragma once
#include <stdint.h>

/*
  Config.h - everything you may need to change for YOUR wiring lives here.
  Flight-logic tuning constants are in FlightCore.h.
*/

// ---------------- Pins (ESP32-C3 Super Mini) ----------------
// Avoid GPIO2, GPIO8, GPIO9 for anything critical: they are strapping pins.
#define PIN_I2C_SDA        6
#define PIN_I2C_SCL        7

#define PIN_RELAY          5      // -> relay module IN pin
#define RELAY_ACTIVE_HIGH  1      // 1 = HIGH energises the relay. See notes: use an ACTIVE-HIGH
                                  // module (or a MOSFET/transistor driver with a pull-DOWN).

#define PIN_LED            8      // Super Mini on-board LED (active LOW)
#define LED_ACTIVE_LOW     1

// OPTIONAL sensor-independent launch detect (the only thing that can start the
// flight timers if BOTH I2C sensors are dead before launch).
// Wire a 'remove-before-flight' style loop: pin is held LOW (to GND) by a wire/jumper
// that is pulled out / broken by the rocket leaving the pad. Pin goes HIGH (internal
// pull-up) = launched. Set to a free GPIO (e.g. 4) to enable, -1 to disable.
#define PIN_LAUNCH_SENSE   -1
#define LAUNCH_SENSE_DEBOUNCE_MS 40
#define LAUNCH_SENSE_TRIP_LEVEL_HIGH 1   // pin goes HIGH (pull-up) when the wire/jumper is pulled = launched

// ---------------- I2C ----------------
#define BMP_ADDR           0x76   // (0x77 is tried automatically as well)
#define MPU_ADDR           0x68
#define I2C_HZ             400000
#define I2C_TIMEOUT_MS     15     // a hung bus can never stall the flight loop longer than this

// ---------------- Relay / ejection output ----------------
#define RELAY_REPEAT_PULSE 1      // 1 = after the first pulse, pulse a 2nd time (redundancy)
#define RELAY_GAP_MS       1000   // gap between the two pulses
#define TEST_COUNTDOWN_MS  3000   // dashboard TEST EJECTION countdown
#define CLICK_PULSE_MS     150    // dashboard "relay click" test

// ---------------- WiFi dashboard ----------------
#define WIFI_SSID          "RocketFC-c3"
#define WIFI_PASS          "00000000"      // >= 8 chars
#define WIFI_OFF_IN_FLIGHT 1                 // radio OFF while flying (less current, no brown-out risk)
#define WIFI_BACK_AFTER_EJECT_MS 8000        // radio comes back after ejection so you can read the peaks

// ---------------- Dashboard display smoothing ----------------
// Time constant (ms) of the smoothing applied to the IMU numbers SHOWN on the dashboard only.
// The flight logic always uses the unsmoothed 200 Hz data. 0 = raw. 40 = barely, 60 = light (default), 120 = calm.
#define DISPLAY_TAU_MS     60

// ---------------- Recovery ----------------
// If the MCU resets mid-flight (brown-out etc.) the flight clock is restored from RTC memory.
// This is added to the stored time to account for the reboot itself.
#define REBOOT_PENALTY_MS  1500

// ---------------- Runtime settings (editable from the dashboard, saved in flash) ----------------
struct Settings {
  uint16_t apogeeTimerS = 12;   // BACKUP: eject this long after launch even if NO sensor says apogee
  uint16_t maxTimerS    = 20;   // ABSOLUTE last resort (must be > apogeeTimerS)
  uint16_t tiltDeg      = 110;  // tilt from pad-vertical that counts as "falling over"
  uint16_t pulseMs      = 2000; // relay ON time per pulse
  uint8_t  launchGx10   = 30;   // launch accel threshold in 0.1 g units (30 = 3.0 g)
};
