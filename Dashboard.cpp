#include "Dashboard.h"
#include "index_html.h"
#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>

static WebServer server(80);
static bool g_running = false;

// ------------------------------------------------------------------ tiny JSON writer
struct Jw {
  char *b; size_t n; size_t len = 0;
  void f(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (len >= n) return;
    va_list ap; va_start(ap, fmt);
    int w = vsnprintf(b + len, n - len, fmt, ap);
    va_end(ap);
    if (w > 0) len += (size_t)w;
    if (len >= n) len = n - 1;
  }
};
static inline float fin(float v) { return isfinite(v) ? v : 0.0f; }

static void handleApi() {
  Snap s;
  if (!readSnapshot(s)) { server.send(503, "text/plain", "no data"); return; }
  static char buf[2048];
  Jw j{buf, sizeof(buf)};
  j.f("{\"ms\":%lu,\"st\":%u,\"rs\":%u,\"ls\":%u,\"bo\":%u,\"io\":%u,\"ry\":%u,\"fc\":%u,\"fl\":%u,",
      (unsigned long)s.ms, s.state, s.reason, s.src, s.baroOk, s.imuOk, s.relayOn, s.fireCount, s.falseLaunches);
  j.f("\"tv\":%u,\"rsm\":%u,\"pe\":%u,\"ps\":%u,\"gc\":%u,\"wn\":%u,\"tr\":%.2f,\"bl\":%.1f,",
      s.tiltValid, s.resumed, s.pinEnabled, s.pinState, s.gyroCalOk, s.warn, fin(s.testRem), fin(s.blindRem));
  j.f("\"alt\":%.2f,\"pk\":%.2f,\"cl\":%.2f,\"pcl\":%.2f,\"pr\":%.2f,\"pp\":%.2f,\"bt\":%.1f,\"it\":%.1f,\"vz\":%.2f,",
      fin(s.alt), fin(s.peakAlt), fin(s.climb), fin(s.peakClimb), fin(s.pressure / 100.0f), fin(s.padPressure / 100.0f),
      fin(s.baroTemp), fin(s.imuTemp), fin(s.vz));
  j.f("\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"mg\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,",
      fin(s.ax), fin(s.ay), fin(s.az), fin(s.mag), fin(s.gx), fin(s.gy), fin(s.gz));
  j.f("\"tl\":%.1f,\"mx\":%.2f,\"mgy\":%.0f,\"mtl\":%.0f,",
      fin(s.tilt), fin(s.maxG), fin(s.maxGyro), fin(s.maxTilt));
  j.f("\"an\":%.1f,\"pn\":%.2f,\"acn\":%.1f,\"gn\":%.2f,\"gb\":%.2f,\"sl\":%.1f,\"ihz\":%.0f,\"bhz\":%.0f,",
      fin(s.altNoiseCm), fin(s.presNoisePa), fin(s.accNoiseMg), fin(s.gyroNoise), fin(s.gyroBias), fin(s.stillS),
      fin(s.imuHz), fin(s.baroHz));
  j.f("\"ie\":%lu,\"be\":%lu,\"rc\":%lu,\"ir\":%lu,\"ft\":%.2f,\"at\":%.2f,\"et\":%.2f,",
      (unsigned long)s.imuErr, (unsigned long)s.baroErr, (unsigned long)s.recov, (unsigned long)s.i2cRecov,
      fin(s.flightT), fin(s.apogeeT), fin(s.ejectT));
  j.f("\"ap\":%u,\"mxt\":%u,\"td\":%u,\"pm\":%u,\"lg\":%u}",
      s.set.apogeeTimerS, s.set.maxTimerS, s.set.tiltDeg, s.set.pulseMs, s.set.launchGx10);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void handleFast() {
  FastSnap f;
  if (!readFast(f)) { server.send(503, "text/plain", "no data"); return; }
  char b[360];
  Jw j{b, sizeof(b)};
  j.f("{\"age\":%lu,\"st\":%u,\"io\":%u,\"bo\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"mg\":%.3f,",
      (unsigned long)(millis() - f.ms), f.state, f.imuOk, f.baroOk, fin(f.ax), fin(f.ay), fin(f.az), fin(f.mag));
  j.f("\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"tl\":%.1f,\"vz\":%.2f,\"alt\":%.2f,\"cl\":%.2f,",
      fin(f.gx), fin(f.gy), fin(f.gz), fin(f.tilt), fin(f.vz), fin(f.alt), fin(f.climb));
  j.f("\"rl\":%.1f,\"pt\":%.1f,\"yw\":%.1f}", fin(f.roll), fin(f.pitch), fin(f.yaw));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

static void handleCmd() {
  String c = server.arg("c");
  Cmd m; memset(&m, 0, sizeof(m));
  m.a = server.arg("a").toInt(); m.b = server.arg("b").toInt();
  m.d = server.arg("d").toInt(); m.e = server.arg("e").toInt();
  if (c == "arm") m.id = CMD_ARM;
  else if (c == "disarm") m.id = CMD_DISARM;
  else if (c == "zero") m.id = CMD_ZERO;
  else if (c == "gyrocal") m.id = CMD_GYROCAL;
  else if (c == "reset") m.id = CMD_RESET;
  else if (c == "peaks") m.id = CMD_RESET_PEAKS;
  else if (c == "test") m.id = CMD_TEST;
  else if (c == "cancel") m.id = CMD_CANCEL;
  else if (c == "click") m.id = CMD_CLICK;
  else if (c == "blind") m.id = CMD_BLIND;
  else if (c == "blindcancel") m.id = CMD_BLIND_CANCEL;
  else if (c == "tiltzero") m.id = CMD_TILTZERO;
  else if (c == "save") { m.id = CMD_SAVE; m.c = server.arg("t").toInt(); }
  else { server.send(400, "application/json", "{\"ok\":0}"); return; }
  bool ok = postCommand(m);
  server.send(200, "application/json", ok ? "{\"ok\":1}" : "{\"ok\":0}");
}

static void startAp() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                         // no modem power-save: lower, steadier latency to the phone
  WiFi.setTxPower(WIFI_POWER_8_5dBm);          // low power: friendlier to a small battery, and required by some C3 Super Minis
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 6, 0, 2);
}

void dashboardBegin() {
  startAp();
  static bool registered = false;               // handlers must be registered ONCE (the radio is restarted after every flight)
  if (!registered) {
    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
    server.on("/api", HTTP_GET, handleApi);
    server.on("/fast", HTTP_GET, handleFast);
    server.on("/cmd", HTTP_POST, handleCmd);
    server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
    registered = true;
  }
  server.begin();
  g_running = true;
}

void dashboardStop() {
  if (!g_running) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_running = false;
}

void dashboardHandle() { if (g_running) server.handleClient(); }
bool dashboardRunning() { return g_running; }
