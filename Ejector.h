#pragma once
#include <Arduino.h>
#include "Config.h"

/*
  Ejector - non-blocking relay driver.

  fire():   pulse #1 (pulseMs) -> gap -> pulse #2 (optional redundancy).
  click():  one short pulse for a "does the relay work" test.
  update(): call every flight-loop iteration; it only compares timestamps, never blocks.

  The pin is driven to the OFF level before it is switched to OUTPUT, and nothing in
  the firmware touches it except this class.
*/
class Ejector {
 public:
  void begin(int pin, bool activeHigh) {
    _pin = pin; _hi = activeHigh;
    digitalWrite(_pin, off());
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, off());
    _phase = IDLE; _on = false;
  }

  bool fire(uint32_t now, uint16_t pulseMs, bool repeat) {
    if (busy()) return false;
    _pulse = pulseMs; _repeat = repeat; _isClick = false;
    _t = now; _phase = P1; set(true);
    _fireCount++;
    return true;
  }

  bool click(uint32_t now, uint16_t ms) {
    if (busy()) return false;
    _pulse = ms; _repeat = false; _isClick = true;
    _t = now; _phase = P1; set(true);
    return true;
  }

  void abort() { set(false); _phase = IDLE; }   // never used in flight; for bench cancel only

  void update(uint32_t now) {
    switch (_phase) {
      case P1:
        if ((uint32_t)(now - _t) >= _pulse) {
          set(false); _t = now;
          _phase = (_repeat && !_isClick) ? GAP : IDLE;
        }
        break;
      case GAP:
        if ((uint32_t)(now - _t) >= RELAY_GAP_MS) { set(true); _t = now; _phase = P2; }
        break;
      case P2:
        if ((uint32_t)(now - _t) >= _pulse) { set(false); _phase = IDLE; }
        break;
      default: break;
    }
  }

  bool busy() const { return _phase != IDLE; }
  bool isOn() const { return _on; }
  uint8_t fireCount() const { return _fireCount; }

 private:
  enum Phase : uint8_t { IDLE, P1, GAP, P2 };
  int _pin = -1; bool _hi = true;
  Phase _phase = IDLE; bool _on = false, _repeat = false, _isClick = false;
  uint16_t _pulse = 0; uint32_t _t = 0; uint8_t _fireCount = 0;
  int off() const { return _hi ? LOW : HIGH; }
  void set(bool on) { _on = on; digitalWrite(_pin, on ? (_hi ? HIGH : LOW) : off()); }
};
