/*
  Smart Precision Agriculture & Intelligent Farm Management System
  Single Raspberry Pi Pico (Wokwi)  -  BCSE312P Assessment 10

  Structure:
    node1Update()  -> soil moisture + irrigation
    node2Update()  -> climate + disease risk + ventilation
    node3Update()  -> security + environmental supervision
    centralCycle() -> polls the 3 nodes every 1 s, validates frames,
                      detects offline nodes, drives buzzer/LED/OLED/Serial

  NOTE: the "frames" are plain C++ structs exchanged inside ONE program.
        They are NOT communication between separate physical controllers.

  Serial commands (type in Serial Monitor + Enter):
    R<0-100> rainfall %      S<0-100> sunlight %     P<0-14> soil pH
    N<0-100> nutrient %      L0/L1 leaf wetness      F1/F2/F3 pick node for failure button
    K = corrupt next Node-2 frame (checksum test)    C1 = reset software values to "normal"
    H = help
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include "DHT.h"

// ---------------- Pins ----------------
const uint8_t PIN_ZA = 26, PIN_ZB = 27, PIN_RES = 28;   // potentiometers
const uint8_t PIN_DHT = 2;
const uint8_t PIN_REQ = 3, PIN_VALID = 4, PIN_INVALID = 5, PIN_ACK = 6, PIN_FAIL = 7;
const uint8_t LED_A[3] = {8, 9, 10};      // Zone A  G,Y,R
const uint8_t LED_B[3] = {11, 12, 13};    // Zone B  G,Y,R
const uint8_t LED_N3 = 14;                // Node 3 security/env LED
const uint8_t LED_C_RED = 15, LED_C_YEL = 19, LED_C_GRN = 20;  // central status
const uint8_t PIN_BUZ = 16, PIN_VALVE = 17, PIN_FLAP = 18;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
DHT dht(PIN_DHT, DHT22);
Servo valveServo, flapServo;
bool oledOK = false;

// ---------------- Levels / names ----------------
enum { LV_NORMAL = 0, LV_WARN = 1, LV_CRIT = 2, LV_OFFLINE = 3 };
const char* LVN[] = {"NORMAL", "WARNING", "CRITICAL", "OFFLINE"};
enum { S_CRITDRY = 0, S_DRY, S_NORMAL, S_WET, S_WATERLOG };

const char* soilName(int c) {
  static const char* n[] = {"CRITICAL DRY", "DRY", "NORMAL", "WET", "WATERLOGGING RISK"};
  return n[c];
}
const char* alertName(int node, int code) {
  static const char* a1[] = {"NONE", "ZONE A DRY", "ZONE B DRY", "ZONE A WET", "ZONE B WET",
                             "ZONE A CRITICAL DRY", "ZONE B CRITICAL DRY",
                             "ZONE A WATERLOGGING RISK", "ZONE B WATERLOGGING RISK", "WATER SHORTAGE"};
  static const char* a2[] = {"NONE", "CLIMATE WARNING", "SOIL PH WARNING", "NUTRIENT WARNING",
                             "DISEASE RISK HIGH", "SOIL PH CRITICAL", "NUTRIENT CRITICAL",
                             "DISEASE RISK SEVERE", "CLIMATE EMERGENCY", "SUNLIGHT CRITICAL"};
  static const char* a3[] = {"NONE", "LOW SUNLIGHT", "HIGH RAINFALL", "LOW RESERVOIR", "SECURITY ALERT"};
  if (code == 100) return "OFFLINE";
  if (node == 0) return a1[code];
  if (node == 1) return a2[code];
  return a3[code];
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ---------------- Frame (simulated communication) ----------------
struct Frame {
  uint8_t  nodeId;     // 1..3
  uint16_t seq;        // sequence number
  uint8_t  status;     // NORMAL/WARNING/CRITICAL
  int16_t  d[5];       // sensor data (meaning depends on node)
  uint8_t  alert;      // alert code (0 = NONE)
  uint32_t ts;         // millis() when generated
  uint8_t  chk;        // XOR checksum
};

// manual prototypes (needed because Arduino auto-prototypes come before the struct)
uint8_t calcChk(const Frame& f);
Frame makeFrame(int i);
bool validateFrame(const Frame& f, int i);

uint8_t calcChk(const Frame& f) {
  uint8_t c = f.nodeId ^ (f.seq & 0xFF) ^ (f.seq >> 8) ^ f.status ^ f.alert;
  for (int i = 0; i < 5; i++) c ^= (f.d[i] & 0xFF) ^ ((f.d[i] >> 8) & 0xFF);
  c ^= (f.ts & 0xFF) ^ ((f.ts >> 8) & 0xFF) ^ ((f.ts >> 16) & 0xFF) ^ ((f.ts >> 24) & 0xFF);
  return c;
}

// what each node publishes every cycle
struct NodeOut { uint8_t status; uint8_t alert; int16_t d[5]; };
NodeOut outp[3];
Frame latest[3];                 // last valid frame per node (central module)
uint16_t seqNo[3] = {0, 0, 0};
uint8_t missed[3] = {0, 0, 0};
bool offline[3] = {false, false, false};
bool nodeFailed[3] = {false, false, false};   // software failure simulation
int failSel = 0;
bool corruptNext = false;
uint8_t prevKey[3] = {0, 0, 0}, prevCrit[3] = {0, 0, 0};

// ---------------- Shared simulated values ----------------
float moist[2] = {50, 50}, res = 80;
float temp = 25, hum = 50;
float rain = 10, sunlight = 55, ph = 6.8, nutrient = 80;
bool leafWet = false;

// ---------------- Event log ----------------
void logEvent(const String& msg) {
  Serial.print("[EVENT t=");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(msg);
}

// ===================================================================
//                       NODE 1  -  SOIL & IRRIGATION
// ===================================================================
int irrZone = -1;                       // -1 = valve closed
uint32_t irrStart = 0, lastIrrEnd = 0;
bool hasEnded = false;
uint16_t cycles[2] = {0, 0};
uint32_t totalMs[2] = {0, 0};
bool shortage = false, postponed = false;
int priorityZone = -1;
String irrMsg = "IDLE", irrReason = "";
int scl[2] = {S_NORMAL, S_NORMAL};

float readPct(uint8_t pin) { return analogRead(pin) * 100.0 / 4095.0; }

int soilClass(float m) {
  if (m < 15) return S_CRITDRY;
  if (m < 30) return S_DRY;
  if (m <= 70) return S_NORMAL;
  if (m <= 85) return S_WET;
  return S_WATERLOG;
}

void setZoneLeds(const uint8_t* leds, int c) {
  digitalWrite(leds[0], c == S_NORMAL);
  digitalWrite(leds[1], c == S_DRY || c == S_WET);
  digitalWrite(leds[2], c == S_CRITDRY || c == S_WATERLOG);
}

void stopIrrigation(const char* why) {
  if (irrZone < 0) return;
  uint32_t d = millis() - irrStart;
  totalMs[irrZone] += d;
  valveServo.write(0);   // CLOSED
  logEvent(String("IRRIGATION STOP: ZONE ") + (irrZone ? "B" : "A") + " (" + why + "), ran " + String(d / 1000.0, 1) + " s, valve CLOSED");
  irrZone = -1;
  lastIrrEnd = millis();
  hasEnded = true;
}

void node1Update() {
  moist[0] = readPct(PIN_ZA);
  moist[1] = readPct(PIN_ZB);
  res = readPct(PIN_RES);
  for (int z = 0; z < 2; z++) scl[z] = soilClass(moist[z]);
  setZoneLeds(LED_A, scl[0]);
  setZoneLeds(LED_B, scl[1]);

  // ---- who needs water? ----
  bool need[2] = {scl[0] <= S_DRY, scl[1] <= S_DRY};
  bool demand = need[0] || need[1];
  priorityZone = -1;
  if (demand) {
    if (need[0] && need[1]) priorityZone = (moist[0] <= moist[1]) ? 0 : 1;   // lower moisture first
    else priorityZone = need[0] ? 0 : 1;
  }
  shortage = demand && res < 15;
  postponed = demand && !shortage && rain > 60;

  // ---- irrigation state machine ----
  irrReason = "";
  if (irrZone >= 0) {
    // already irrigating: check every shutdown condition
    if (res < 15) stopIrrigation("reservoir below 15%");
    else if (rain > 60) stopIrrigation("rain probability above 60%");
    else if (scl[irrZone] >= S_WET) stopIrrigation("soil wet");
    else if (moist[irrZone] >= 40) stopIrrigation("hysteresis target 40% reached");
    else if (millis() - irrStart >= 10000) stopIrrigation("10 s limit");
  } else if (demand && !shortage && !postponed) {
    bool cool = !hasEnded || (millis() - lastIrrEnd >= 5000);
    if (cool) {
      irrZone = priorityZone;
      irrStart = millis();
      cycles[irrZone]++;
      valveServo.write(90);   // OPEN
      logEvent(String("IRRIGATION START: ZONE ") + (irrZone ? "B" : "A") + " at " + (int)moist[irrZone] + "%, valve OPEN");
    }
  }

  // ---- text for Serial ----
  if (irrZone >= 0) irrMsg = String("ZONE ") + (irrZone ? "B" : "A") + " SELECTED";
  else if (shortage) { irrMsg = "INHIBITED"; irrReason = "water shortage (reservoir < 15%)"; }
  else if (postponed) { irrMsg = "POSTPONED"; irrReason = "rain probability > 60%"; }
  else if (demand) { irrMsg = "COOLDOWN"; irrReason = "waiting 5 s between cycles"; }
  else if (scl[0] >= S_WET || scl[1] >= S_WET) { irrMsg = "INHIBITED"; irrReason = "soil wet / waterlogged"; }
  else irrMsg = "IDLE";

  // ---- status + alert code ----
  uint8_t alert = 0;
  if (shortage) alert = 9;
  else if (scl[0] == S_WATERLOG) alert = 7;
  else if (scl[1] == S_WATERLOG) alert = 8;
  else if (scl[0] == S_CRITDRY) alert = 5;
  else if (scl[1] == S_CRITDRY) alert = 6;
  else if (scl[0] == S_WET) alert = 3;
  else if (scl[1] == S_WET) alert = 4;
  else if (scl[0] == S_DRY) alert = 1;
  else if (scl[1] == S_DRY) alert = 2;

  uint8_t st = LV_NORMAL;
  if (alert == 1 || alert == 2 || alert == 3 || alert == 4 || postponed) st = LV_WARN;
  if (alert >= 5) st = LV_CRIT;

  outp[0].status = st;
  outp[0].alert = alert;
  outp[0].d[0] = (int16_t)moist[0];
  outp[0].d[1] = (int16_t)moist[1];
  outp[0].d[2] = (int16_t)res;
  outp[0].d[3] = irrZone + 1;        // 0 closed, 1 = zone A open, 2 = zone B open
  outp[0].d[4] = priorityZone + 1;   // 0 none
}

// ===================================================================
//                  NODE 2  -  CLIMATE & DISEASE RISK
// ===================================================================
bool flapOpen = false;
int diseaseScoreV = 0;
String factors = "none";
bool climateCrit = false;

int cTemp(float t) { if (t > 38) return LV_CRIT; if (t > 30 || t < 18) return LV_WARN; return LV_NORMAL; }
int cHum(float h)  { if (h > 90) return LV_CRIT; if (h > 75 || h < 40) return LV_WARN; return LV_NORMAL; }
int cSun(float s)  { if (s < 10 || s > 95) return LV_CRIT; if (s < 30 || s > 80) return LV_WARN; return LV_NORMAL; }
int cPh(float p)   { if (p < 5.0 || p > 8.5) return LV_CRIT; if (p < 6.0 || p > 7.5) return LV_WARN; return LV_NORMAL; }
int cNut(float n)  { if (n < 30) return LV_CRIT; if (n <= 60) return LV_WARN; return LV_NORMAL; }

const char* diseaseCat(int s) {
  if (s < 30) return "LOW";
  if (s < 60) return "MODERATE";
  if (s < 80) return "HIGH";
  return "SEVERE";
}

int calcDisease() {
  int s = 0;
  factors = "";
  if (temp > 30) { s += 30; factors += "temp>30C(+30) "; }
  if (hum > 80)  { s += 30; factors += "humidity>80%(+30) "; }
  if (leafWet)   { s += 25; factors += "leaf-wetness(+25) "; }
  if (hum > 90)  { s += 15; factors += "humidity>90%(+15) "; }
  if (s > 100) s = 100;
  if (factors == "") factors = "none";
  return s;
}

void node2Update() {
  float t = dht.readTemperature(), h = dht.readHumidity();
  if (!isnan(t)) temp = t;
  if (!isnan(h)) hum = h;

  int lt = cTemp(temp), lh = cHum(hum), ls = cSun(sunlight), lp = cPh(ph), ln = cNut(nutrient);
  diseaseScoreV = calcDisease();
  climateCrit = (lt == LV_CRIT || lh == LV_CRIT);

  // ventilation flap with hysteresis (open > 35, close < 32)
  if (temp > 35 && !flapOpen) { flapOpen = true; flapServo.write(90); logEvent("VENTILATION FLAP OPENED (temp > 35C)"); }
  else if (temp < 32 && flapOpen) { flapOpen = false; flapServo.write(0); logEvent("VENTILATION FLAP CLOSED (temp < 32C)"); }

  uint8_t alert = 0;
  if (climateCrit) alert = 8;
  else if (diseaseScoreV >= 80) alert = 7;
  else if (ln == LV_CRIT) alert = 6;
  else if (lp == LV_CRIT) alert = 5;
  else if (ls == LV_CRIT) alert = 9;
  else if (diseaseScoreV >= 60) alert = 4;
  else if (ln == LV_WARN) alert = 3;
  else if (lp == LV_WARN) alert = 2;
  else if (lt == LV_WARN || lh == LV_WARN || ls == LV_WARN) alert = 1;

  uint8_t st = LV_NORMAL;
  if (alert >= 1 && alert <= 4) st = LV_WARN;
  if (alert >= 5) st = LV_CRIT;

  outp[1].status = st;
  outp[1].alert = alert;
  outp[1].d[0] = (int16_t)(temp * 10);
  outp[1].d[1] = (int16_t)(hum * 10);
  outp[1].d[2] = (int16_t)sunlight;
  outp[1].d[3] = diseaseScoreV;
  outp[1].d[4] = flapOpen;
}

// ===================================================================
//                  NODE 3  -  SECURITY & ENVIRONMENT
// ===================================================================
bool sessionActive = false, gateOpen = false, secAlert = false;
uint8_t failedCnt = 0;
uint32_t gateUntil = 0;
int envIndex = 0;
String envAlerts = "none";

void accessRequest() {
  if (secAlert) { logEvent("ACCESS REQUEST DENIED - security lock active"); return; }
  sessionActive = true;
  logEvent("ACCESS REQUEST - session started, enter code");
}
void validCode() {
  if (secAlert) { logEvent("VALID CODE IGNORED - security lock active (hold ACK 3 s to recover)"); return; }
  if (!sessionActive) { logEvent("CODE ENTERED WITHOUT ACCESS SESSION - ignored"); return; }
  gateOpen = true; gateUntil = millis() + 5000;
  failedCnt = 0; sessionActive = false;
  logEvent("ACCESS GRANTED - gate UNLOCKED for 5 s");
}
void invalidCode() {
  if (!sessionActive) { logEvent("CODE ENTERED WITHOUT ACCESS SESSION - ignored"); return; }
  failedCnt++;
  logEvent(String("ACCESS VIOLATION - invalid code (") + failedCnt + "/3)");
  if (failedCnt >= 3 && !secAlert) {
    secAlert = true; sessionActive = false; gateOpen = false;
    logEvent("SECURITY ALERT - 3 consecutive invalid codes, gate LOCKED");
  }
}
void recoverSecurity() {
  if (!secAlert) { logEvent("RECOVERY ignored - no security alert"); return; }
  secAlert = false; failedCnt = 0; sessionActive = false;
  logEvent("RECOVERY - security alert cleared by authorised reset (ACK held 3 s)");
}

void node3Update() {
  envIndex = (int)(rain * 0.4 + (100 - sunlight) * 0.3 + (100 - res) * 0.3);
  envAlerts = "";
  if (rain > 60) envAlerts += "high-rainfall ";
  if (sunlight < 30) envAlerts += "low-sunlight ";
  if (res < 15) envAlerts += "low-reservoir ";
  if (envAlerts == "") envAlerts = "none";

  uint8_t alert = 0;
  if (secAlert) alert = 4;
  else if (res < 15) alert = 3;
  else if (rain > 60) alert = 2;
  else if (sunlight < 30) alert = 1;

  outp[2].status = secAlert ? LV_CRIT : (alert ? LV_WARN : LV_NORMAL);
  outp[2].alert = alert;
  outp[2].d[0] = gateOpen;
  outp[2].d[1] = failedCnt;
  outp[2].d[2] = envIndex;
  outp[2].d[3] = (int16_t)rain;
  outp[2].d[4] = secAlert;
}

// ===================================================================
//                       NODE 4  -  CENTRAL CONTROL
// ===================================================================
uint32_t pollCount = 0, nextTick = 0, ackUntil = 0;
int buzLevel = 0;      // 0 silent, 1 slow, 2 fast, 3 continuous
int overall = 0;       // 0 normal, 1 attention, 2 emergency
int cN = 0, cW = 0, cC = 0, cO = 0;

Frame makeFrame(int i) {
  Frame f;
  f.nodeId = i + 1;
  f.seq = ++seqNo[i];
  f.status = outp[i].status;
  for (int k = 0; k < 5; k++) f.d[k] = outp[i].d[k];
  f.alert = outp[i].alert;
  f.ts = millis();
  f.chk = calcChk(f);
  return f;
}
bool validateFrame(const Frame& f, int i) {
  if (f.nodeId != i + 1) return false;
  if (f.status > LV_CRIT) return false;
  if (f.chk != calcChk(f)) return false;
  return true;
}

void pollNode(int i) {
  bool ok = false;
  Frame f;
  memset(&f, 0, sizeof(f));
  if (!nodeFailed[i]) {
    f = makeFrame(i);
    if (corruptNext && i == 1) { f.chk ^= 0xFF; corruptNext = false; }
    ok = validateFrame(f, i);
    if (!ok) logEvent(String("NODE ") + (i + 1) + " FRAME REJECTED (bad checksum)");
  }
  if (ok) {
    if (offline[i]) { offline[i] = false; logEvent(String("NODE ") + (i + 1) + " RECOVERED - frame validated, status restored"); }
    missed[i] = 0;
    latest[i] = f;
  } else {
    missed[i]++;
    if (missed[i] >= 3 && !offline[i]) {
      offline[i] = true;
      logEvent(String("NODE ") + (i + 1) + " OFFLINE - 3 missed polls (CRITICAL)");
    }
  }
}

int effStatus(int i) { return offline[i] ? LV_OFFLINE : latest[i].status; }

void ackBuzzer() {
  ackUntil = millis() + 30000;
  logEvent("ALARM ACKNOWLEDGED - buzzer silenced for 30 s");
  Serial.println("BUZZER ACKNOWLEDGED FOR 30 SECONDS");
}

void drawOled();
void printCycle();

void centralCycle() {
  pollCount++;

  // run the node modules (a failed node does not run / does not answer)
  if (nodeFailed[0]) stopIrrigation("NODE 1 failed - fail-safe close"); else node1Update();
  if (!nodeFailed[1]) node2Update();
  if (!nodeFailed[2]) node3Update();

  // poll + validate
  for (int i = 0; i < 3; i++) pollNode(i);

  // count statuses
  cN = cW = cC = cO = 0;
  for (int i = 0; i < 3; i++) {
    int s = effStatus(i);
    if (s == LV_NORMAL) cN++; else if (s == LV_WARN) cW++; else if (s == LV_CRIT) cC++; else cO++;
  }
  overall = (cC > 0 || cO > 0) ? 2 : (cW > 0 ? 1 : 0);

  // alert change logging + new-critical detection
  bool newCrit = false;
  for (int i = 0; i < 3; i++) {
    uint8_t key = offline[i] ? 100 : (latest[i].status == LV_NORMAL ? 0 : latest[i].alert);
    if (key != prevKey[i] && key != 100) {
      if (key == 0) logEvent(String("NODE ") + (i + 1) + " alert cleared");
      else logEvent(String("NODE ") + (i + 1) + " ALERT: " + alertName(i, key));
    }
    bool crit = offline[i] || latest[i].status == LV_CRIT;
    uint8_t ck = crit ? key : 0;
    if (ck != 0 && ck != prevCrit[i]) newCrit = true;
    prevCrit[i] = ck;
    prevKey[i] = key;
  }
  if (newCrit && ackUntil != 0) { ackUntil = 0; logEvent("ACKNOWLEDGEMENT CANCELLED - new critical event"); }
  if (ackUntil != 0 && millis() >= ackUntil) { ackUntil = 0; logEvent("ACK period expired"); }

  // buzzer priority
  buzLevel = 0;
  for (int i = 0; i < 3; i++) {
    int lv = 0;
    if (offline[i]) lv = 3;
    else if (latest[i].status == LV_CRIT) {
      if ((i == 0 && latest[i].alert == 9) || i == 2) lv = 3;   // severe water shortage / security alert
      else lv = 2;
    } else if (latest[i].status == LV_WARN) lv = 1;
    if (lv > buzLevel) buzLevel = lv;
  }

  // central LEDs
  digitalWrite(LED_C_GRN, overall == 0);
  digitalWrite(LED_C_YEL, overall == 1);
  digitalWrite(LED_C_RED, overall == 2);

  printCycle();
  drawOled();
}

void printCycle() {
  const Frame &a = latest[0], &b = latest[1], &c = latest[2];
  Serial.println();
  Serial.printf("=============== POLL #%lu | t=%.3f s ===============\n", (unsigned long)pollCount, millis() / 1000.0);

  // 1,2 soil + irrigation demand
  Serial.printf("[NODE1] Zone A: %d%% (%s) | Zone B: %d%% (%s) | Reservoir: %d%%\n",
                a.d[0], soilName(soilClass(a.d[0])), a.d[1], soilName(soilClass(a.d[1])), a.d[2]);
  Serial.printf("[NODE1] Irrigation demand: %s\n", priorityZone < 0 ? "NONE" : (priorityZone ? "ZONE B" : "ZONE A"));
  if (priorityZone >= 0 && (scl[0] <= S_DRY) && (scl[1] <= S_DRY))
    Serial.printf("IRRIGATION PRIORITY: ZONE %s\n", priorityZone ? "B" : "A");
  Serial.printf("IRRIGATION: %s %s%s%s\n", irrMsg.c_str(), irrReason.length() ? "(" : "", irrReason.c_str(), irrReason.length() ? ")" : "");
  float dur = irrZone >= 0 ? (millis() - irrStart) / 1000.0 : 0;
  Serial.printf("[VALVE] %s | current run: %.1f s | Zone A: %u cycles, %.1f s total | Zone B: %u cycles, %.1f s total\n",
                irrZone >= 0 ? "OPEN" : "CLOSED", dur,
                cycles[0], (totalMs[0] + (irrZone == 0 ? millis() - irrStart : 0)) / 1000.0,
                cycles[1], (totalMs[1] + (irrZone == 1 ? millis() - irrStart : 0)) / 1000.0);

  // 3,4,5 climate / crop health
  Serial.printf("[NODE2] Temp: %.1f C | Humidity: %.1f%% | Sunlight: %d%% | Flap: %s\n",
                b.d[0] / 10.0, b.d[1] / 10.0, b.d[2], b.d[4] ? "OPEN" : "CLOSED");
  Serial.printf("[NODE2] Rainfall(sim): %.0f%% | Soil pH(sim): %.1f | Nutrient(sim): %.0f%% | Leaf wet: %s\n",
                rain, ph, nutrient, leafWet ? "YES" : "NO");
  Serial.printf("[NODE2] Disease risk score: %d (%s) | Factors: %s\n", b.d[3], diseaseCat(b.d[3]), factors.c_str());

  // 7 access
  Serial.printf("[NODE3] Gate: %s | Failed attempts: %d/3 | Env risk index: %d | Env alerts: %s\n",
                c.d[0] ? "UNLOCKED" : "LOCKED", c.d[1], c.d[2], envAlerts.c_str());

  // 8 node frames
  for (int i = 0; i < 3; i++) {
    Serial.printf("[FRAME] N%d %s seq=%u status=%s alert=%s ts=%lu chk=0x%02X missed=%d\n",
                  i + 1, offline[i] ? "OFFLINE" : "ONLINE ", latest[i].seq, LVN[latest[i].status],
                  alertName(i, latest[i].alert), (unsigned long)latest[i].ts, latest[i].chk, missed[i]);
  }
  // 9 counts
  Serial.printf("[COUNT] NORMAL=%d WARNING=%d CRITICAL=%d OFFLINE=%d\n", cN, cW, cC, cO);

  // active alerts (current state) - events above/below are history
  Serial.println("[ACTIVE ALERTS]");
  bool any = false;
  for (int i = 0; i < 3; i++) {
    if (offline[i]) { Serial.printf("  NODE %d: OFFLINE\n", i + 1); any = true; }
    else if (latest[i].status != LV_NORMAL) { Serial.printf("  NODE %d: %s\n", i + 1, alertName(i, latest[i].alert)); any = true; }
  }
  if (!any) Serial.println("  none");

  // buzzer / ack
  const char* bz[] = {"SILENT", "SLOW BEEP (warning)", "FAST BEEP (critical)", "CONTINUOUS (emergency)"};
  if (millis() < ackUntil) Serial.printf("[BUZZER] %s | ACK ACTIVE, %lu s left -> muted\n", bz[buzLevel], (unsigned long)((ackUntil - millis()) / 1000));
  else Serial.printf("[BUZZER] %s | ACK: not active\n", bz[buzLevel]);

  // 10 overall
  if (overall == 0) Serial.println("FARM STATUS: ALL NORMAL");
  else if (overall == 1) Serial.println("FARM STATUS: ATTENTION REQUIRED");
  else Serial.println("FARM STATUS: EMERGENCY");
}

// ---------------- OLED (rotating pages, 3 s each) ----------------
void drawOled() {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  int page = (pollCount / 3) % 5;
  const Frame &a = latest[0], &b = latest[1], &c = latest[2];

  if (page == 0) {
    oled.println("FARM STATUS:");
    oled.setTextSize(overall == 1 ? 1 : 2);
    oled.println(overall == 0 ? "ALL NORMAL" : (overall == 1 ? "ATTENTION REQUIRED" : "EMERGENCY"));
    oled.setTextSize(1);
    oled.setCursor(0, 36);
    oled.printf("N:%d W:%d C:%d O:%d\n", cN, cW, cC, cO);
    const char* bz[] = {"Buzzer: silent", "Buzzer: slow", "Buzzer: fast", "Buzzer: CONTINUOUS"};
    oled.println(bz[buzLevel]);
    if (millis() < ackUntil) oled.printf("ACK %lus left\n", (unsigned long)((ackUntil - millis()) / 1000));
  } else if (page == 1) {
    oled.println("-- SOIL / IRRIGATION --");
    oled.printf("Zone A: %d%% %s\n", a.d[0], soilName(soilClass(a.d[0])));
    oled.printf("Zone B: %d%% %s\n", a.d[1], soilName(soilClass(a.d[1])));
    oled.printf("Reservoir: %d%%\n", a.d[2]);
    oled.printf("Valve: %s\n", a.d[3] == 0 ? "CLOSED" : (a.d[3] == 1 ? "OPEN-Zone A" : "OPEN-Zone B"));
    if (offline[0]) oled.println("(Node1 OFFLINE-old)");
  } else if (page == 2) {
    oled.println("-- CLIMATE / CROP --");
    oled.printf("Temp: %.1fC Hum:%.0f%%\n", b.d[0] / 10.0, b.d[1] / 10.0);
    oled.printf("Sunlight: %d%%\n", b.d[2]);
    oled.printf("Disease: %d %s\n", b.d[3], diseaseCat(b.d[3]));
    oled.printf("Flap: %s\n", b.d[4] ? "OPEN" : "CLOSED");
    if (offline[1]) oled.println("(Node2 OFFLINE-old)");
  } else if (page == 3) {
    oled.println("-- ACCESS / ENV --");
    oled.printf("Gate: %s\n", c.d[0] ? "UNLOCKED" : "LOCKED");
    oled.printf("Failed codes: %d/3\n", c.d[1]);
    oled.printf("Security: %s\n", c.d[4] ? "ALERT" : "OK");
    oled.printf("Rain:%.0f%% pH:%.1f\n", rain, ph);
    oled.printf("Nutrient: %.0f%%\n", nutrient);
    if (offline[2]) oled.println("(Node3 OFFLINE-old)");
  } else {
    oled.println("-- NODES / ALERTS --");
    for (int i = 0; i < 3; i++)
      oled.printf("N%d %s %s\n", i + 1, offline[i] ? "OFFLINE" : "ONLINE", LVN[effStatus(i) == LV_OFFLINE ? 3 : effStatus(i)]);
    for (int i = 0; i < 3; i++) {
      if (offline[i]) oled.printf("!N%d OFFLINE\n", i + 1);
      else if (latest[i].status != LV_NORMAL) oled.printf("!N%d %s\n", i + 1, alertName(i, latest[i].alert));
    }
  }
  oled.display();
}

// ---------------- Buttons ----------------
struct Btn { uint8_t pin; bool state; uint32_t t; };

// manual prototype (same reason as Frame above)
int btnEdge(Btn& b);

Btn bReq = {PIN_REQ, true, 0}, bValid = {PIN_VALID, true, 0}, bInvalid = {PIN_INVALID, true, 0},
    bAck = {PIN_ACK, true, 0}, bFail = {PIN_FAIL, true, 0};
uint32_t ackDownAt = 0;

int btnEdge(Btn& b) {          // 1 = pressed, 2 = released, 0 = nothing
  bool r = digitalRead(b.pin);
  if (r != b.state && millis() - b.t > 30) {
    b.t = millis();
    b.state = r;
    return r ? 2 : 1;
  }
  return 0;
}

void handleButtons() {
  if (btnEdge(bReq) == 1) accessRequest();
  if (btnEdge(bValid) == 1) validCode();
  if (btnEdge(bInvalid) == 1) invalidCode();
  if (btnEdge(bFail) == 1) {
    nodeFailed[failSel] = !nodeFailed[failSel];
    logEvent(String("NODE ") + (failSel + 1) + " FAILURE SIMULATION " + (nodeFailed[failSel] ? "ON" : "OFF"));
  }
  int e = btnEdge(bAck);
  if (e == 1) ackDownAt = millis();
  if (e == 2) {
    if (millis() - ackDownAt >= 3000) recoverSecurity();   // long press = authorised recovery
    else ackBuzzer();                                      // short press = acknowledge
  }
}

// ---------------- Serial commands ----------------
void printHelp() {
  Serial.println("Commands: R<0-100> rain | S<0-100> sun | P<0-14> pH | N<0-100> nutrient | L0/L1 leaf wet");
  Serial.println("          F1/F2/F3 choose node for failure button | K corrupt Node-2 frame | C1 normal values | H help");
  Serial.println("Buttons : GP3 request, GP4 valid code, GP5 invalid code, GP6 ACK (hold 3 s = recover security), GP7 node failure");
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  s.toUpperCase();
  if (s.length() == 0) return;
  char c = s[0];
  float v = s.substring(1).toFloat();
  switch (c) {
    case 'R': rain = clampf(v, 0, 100); break;
    case 'S': sunlight = clampf(v, 0, 100); break;
    case 'P': ph = clampf(v, 0, 14); break;
    case 'N': nutrient = clampf(v, 0, 100); break;
    case 'L': leafWet = (v != 0); break;
    case 'F': if (v >= 1 && v <= 3) failSel = (int)v - 1; break;
    case 'K': corruptNext = true; break;
    case 'C': if (v == 1) { rain = 10; sunlight = 55; ph = 6.8; nutrient = 80; leafWet = false; } break;
    case 'H': printHelp(); break;
    default: Serial.println("[CMD] unknown (type H)"); return;
  }
  Serial.print("[CMD] ok: ");
  Serial.println(s);
}

// ---------------- Buzzer + misc outputs ----------------
void updateBuzzer() {
  static bool prev = false;
  uint32_t now = millis();
  bool on = false;
  if (now >= ackUntil) {
    if (buzLevel == 3) on = true;
    else if (buzLevel == 2) on = (now % 300) < 150;
    else if (buzLevel == 1) on = (now % 1500) < 150;
  }
  if (on != prev) {
    if (on) tone(PIN_BUZ, 2000); else noTone(PIN_BUZ);
    prev = on;
  }
}

void updateNode3Led() {
  digitalWrite(LED_N3, secAlert || climateCrit || outp[2].status == LV_CRIT);
}

void updateGate() {
  if (gateOpen && millis() >= gateUntil) { gateOpen = false; logEvent("GATE RELOCKED (5 s elapsed)"); }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.setTimeout(50);

  analogReadResolution(12);

  for (int i = 0; i < 3; i++) { pinMode(LED_A[i], OUTPUT); pinMode(LED_B[i], OUTPUT); }
  pinMode(LED_N3, OUTPUT);
  pinMode(LED_C_RED, OUTPUT); pinMode(LED_C_YEL, OUTPUT); pinMode(LED_C_GRN, OUTPUT);
  pinMode(PIN_BUZ, OUTPUT);
  pinMode(PIN_REQ, INPUT_PULLUP); pinMode(PIN_VALID, INPUT_PULLUP); pinMode(PIN_INVALID, INPUT_PULLUP);
  pinMode(PIN_ACK, INPUT_PULLUP); pinMode(PIN_FAIL, INPUT_PULLUP);

  valveServo.attach(PIN_VALVE);
  flapServo.attach(PIN_FLAP);
  valveServo.write(0);
  flapServo.write(0);

  dht.begin();
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Smart Farm System");
    oled.println("Starting...");
    oled.display();
  }

  Serial.println("=== Smart Precision Agriculture & Farm Management (Pico / Wokwi) ===");
  Serial.println("Frames are data structures inside ONE program (not separate controllers).");
  printHelp();
  delay(2000);                       // let the DHT22 produce its first reading
  nextTick = millis() + 1000;
}

void loop() {
  handleSerial();
  handleButtons();
  updateGate();
  updateNode3Led();
  updateBuzzer();
  if (millis() >= nextTick) {
    nextTick += 1000;
    centralCycle();
  }
  delay(1);   // speeds up the simulation
}