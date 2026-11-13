#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include "FlightCore.h"

/*
  RtcStore - keeps the in-flight state in RTC memory (survives watchdog, brown-out and
  software resets, NOT a full power cycle). If the chip reboots mid-flight, setup()
  restores the flight clock + peaks and carries on, instead of waking up on the pad
  with no idea that it is already airborne.
*/
struct RtcRecord {
  uint32_t magic;
  FlightSave save;
  uint32_t crc;
};

static RTC_NOINIT_ATTR RtcRecord g_rtc;
// Tied to the build date/time: re-flashing new firmware can NEVER resurrect an old flight record.
static constexpr uint32_t hashStr(const char *s, uint32_t h = 2166136261u) {
  return *s ? hashStr(s + 1, (h ^ (uint8_t)*s) * 16777619u) : h;
}
static const uint32_t RTC_MAGIC = 0xF11647E5u ^ hashStr(__DATE__ __TIME__);

static inline uint32_t rtcCrc(const RtcRecord &r) {
  const uint8_t *p = (const uint8_t *)&r;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(RtcRecord, crc); i++) { h ^= p[i]; h *= 16777619u; }
  return h;
}

static inline void rtcClear() { memset((void *)&g_rtc, 0, sizeof(g_rtc)); }

static inline void rtcSave(const FlightCore &c, uint32_t now) {
  RtcRecord r;
  memset(&r, 0, sizeof(r));
  r.magic = RTC_MAGIC;
  c.exportSave(now, r.save);
  r.crc = rtcCrc(r);
  memcpy((void *)&g_rtc, &r, sizeof(r));     // memcpy (not '=') so padding bytes can't differ from what the CRC saw
}

// true + filled if a plausible record from a PREVIOUS flight run exists and the reset was not a cold power-on
static inline bool rtcLoad(FlightSave &out) {
  if (esp_reset_reason() == ESP_RST_POWERON) { rtcClear(); return false; }
  if (g_rtc.magic != RTC_MAGIC || g_rtc.crc != rtcCrc(g_rtc)) return false;
  uint8_t st = g_rtc.save.state;
  if (st != (uint8_t)FState::FLIGHT && st != (uint8_t)FState::EJECTED) return false;
  out = g_rtc.save;
  return true;
}
