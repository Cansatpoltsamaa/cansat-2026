// =======================================================
// MAJAAM 2026 – Minimal receiver + GS timestamp + TIME/START/START_ALT forward
// ESP32 Arduino core 3.3.7 (IDF5) – uued ESP-NOW callbackid
// =======================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <time.h>
#include <sys/time.h>

// ---- CanSati STA MAC (PANE ÕIGE!) ----
uint8_t CANSAT_MAC[6] = {0xD4, 0x8C, 0x49, 0xCA, 0x29, 0x40};  // <-- MUUDA VASTAVAKS

// ---- WiFi kanal (sama mis CanSat) ----
#define WIFI_CH 1

// -------------------------------------------------------
// Abifunktsioon: kas RTC tundub seatud?
// -------------------------------------------------------
static bool rtcLooksValid() {
  struct timeval tv; gettimeofday(&tv, nullptr);
  return (tv.tv_sec > 978307200); // >2001‑01‑01
}

// -------------------------------------------------------
// GS timestamp string
// -------------------------------------------------------
static void gsTimestamp(char* out, size_t n) {
  if (rtcLooksValid()) {
    struct timeval tv; gettimeofday(&tv, nullptr);
    time_t sec = tv.tv_sec;
    int ms = tv.tv_usec / 1000;
    struct tm tmv; localtime_r(&sec, &tmv);
    snprintf(out, n, "%04d/%02d/%02d %02d:%02d:%02d.%03d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
  } else {
    uint64_t ms = esp_timer_get_time() / 1000ULL;
    snprintf(out, n, "T+%llu ms", (unsigned long long)ms);
  }
}

// -------------------------------------------------------
// Serial realugemine
// -------------------------------------------------------
String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { String out = buf; buf = ""; return out; }
    buf += c;
  }
  return "";
}

// -------------------------------------------------------
// RTC seadmine parsimisega
// -------------------------------------------------------
bool parseAndSetRTC(const char* s) {
  int Y,M,D,h,m,sc,ms=0;
  int n = sscanf(s, "%d/%d/%d %d:%d:%d.%d", &Y,&M,&D,&h,&m,&sc,&ms);
  if (n < 6) return false;

  struct tm tmv = {};
  tmv.tm_year = Y - 1900;
  tmv.tm_mon  = M - 1;
  tmv.tm_mday = D;
  tmv.tm_hour = h;
  tmv.tm_min  = m;
  tmv.tm_sec  = sc;

  time_t t = mktime(&tmv);
  if (t <= 0) return false;

  struct timeval tv;
  tv.tv_sec  = t;
  tv.tv_usec = ms * 1000;
  settimeofday(&tv, nullptr);
  return true;
}

// -------------------------------------------------------
// ESP-NOW edastuse abi
// -------------------------------------------------------
static void sendToCanSat(const char* s) {
  if (!s) return;
  esp_err_t err = esp_now_send(CANSAT_MAC, (const uint8_t*)s, strlen(s));
  if (err != ESP_OK) {
    Serial.print("[GS] ESP-NOW send viga: "); Serial.println((int)err);
  }
}

// -------------------------------------------------------
// ESP-NOW: CanSat -> GS (vastuvõtt)
// -------------------------------------------------------
void onNowRecv(const esp_now_recv_info_t *info,
               const uint8_t *data,
               int len)
{
  char ts[40];
  gsTimestamp(ts, sizeof(ts));

  Serial.print(ts);
  Serial.print(" | ");
  if (len > 0 && data) Serial.write(data, len);

  // Kui CanSati rida ei lõppe \n, lisa see
  if (len == 0 || data[len - 1] != '\n')
    Serial.println();
}

// -------------------------------------------------------
// ESP-NOW: saatmise staatus (valikuline logi)
// -------------------------------------------------------
void onNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[GS] TX OK" : "[GS] TX FAIL");
  (void)info; (void)status;
}

// -------------------------------------------------------
// Serial käsud: TIME, START, START_ALT -> forward CanSatile
// -------------------------------------------------------
void handleSerial() {
  String L = readLine();
  if (L.length() == 0) return;

  L.trim();
  String cmd = L;
  String arg = "";

  int sp = L.indexOf(' ');
  if (sp > 0) {
    cmd = L.substring(0, sp);
    arg = L.substring(sp + 1);
  }
  cmd.toUpperCase();

  if (cmd == "TIME") {
    if (parseAndSetRTC(arg.c_str())) {
      Serial.print("[GS] RTC seatud -> "); Serial.println(arg);

      // Saadame TIME CanSat'ile kujul: "TIME,YYYY/MM/DD HH:MM:SS.mmm"
      char buf[96];
      snprintf(buf, sizeof(buf), "TIME,%s", arg.c_str());
      sendToCanSat(buf);
      Serial.println("[GS] TIME edasi CanSatile");
    } else {
      Serial.println("[GS] TIME käsu parsimise viga! Formaat: YYYY/MM/DD HH:MM:SS.mmm");
    }
    return;
  }

  if (cmd == "START") {
    sendToCanSat("START");
    Serial.println("[GS] START edasi CanSatile");
    return;
  }

  if (cmd == "START_ALT") {
    sendToCanSat("START_ALT");
    Serial.println("[GS] START_ALT edasi CanSatile");
    return;
  }

  Serial.println("[GS] Käsud: TIME YYYY/MM/DD HH:MM:SS.mmm | START | START_ALT");
}

// -------------------------------------------------------
// SETUP
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(150);

  Serial.println("\n[MAJAAM] Vastuvõtja + GS timestamp + TIME/START forward");

  // Ajavöönd (Eesti)
  setenv("TZ","EET-2EEST,M3.5.0/3,M10.5.0/4",1);
  tzset();

  // WiFi STA + kanal
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init FAIL");
    return;
  }

  esp_now_register_recv_cb(onNowRecv);
  esp_now_register_send_cb(onNowSent);

  // Lisa CanSat peer (obligatoorne, et saaks saata)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CANSAT_MAC, 6);
  peer.channel = WIFI_CH;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peer) == ESP_OK)
    Serial.println("[OK] Peer lisatud");
  else
    Serial.println("[ERR] Peer lisamine ebaõnnestus");

  Serial.println("[MAJAAM] Ootan telemeetriat...");
  Serial.println("[MAJAAM] TIME kasutamiseks: TIME YYYY/MM/DD HH:MM:SS.mmm");
}

// -------------------------------------------------------
// LOOP
// -------------------------------------------------------
void loop() {
  handleSerial();
}