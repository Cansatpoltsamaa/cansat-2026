#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <sys/time.h>
#include <SD.h>
#include <SPI.h>

// ---- CAN-SAT MAC (asenda oma CanSati STA MAC'iga!) ----
uint8_t CANSAT_MAC[6] = { 0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF };   // <-- MUUDA!

// ---- SD SEADISTUS ----
SPIClass spiSD(VSPI);
static const int SD_CS = 13;
File logFile;
bool sd_ok = false;

// ---- SERIAL INPUT BUFFER ----
String readSerialLine() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = buf;
      buf = "";
      return out;
    }
    buf += c;
  }
  return "";
}

// ---------- RTC SEADISTUS SERIALI KAUDU ----------
bool setRTC_fromString(const String &ts) {
  int Y, M, D, h, m, s, ms = 0;
  int n = sscanf(ts.c_str(),
                 "%d-%d-%d %d:%d:%d.%d",
                 &Y, &M, &D, &h, &m, &s, &ms);

  if (n < 6) return false;

  struct tm tmv = {};
  tmv.tm_year = Y - 1900;
  tmv.tm_mon  = M - 1;
  tmv.tm_mday = D;
  tmv.tm_hour = h;
  tmv.tm_min  = m;
  tmv.tm_sec  = s;

  time_t t = mktime(&tmv);
  if (t <= 0) return false;

  struct timeval tv;
  tv.tv_sec  = t;
  tv.tv_usec = ms * 1000;

  settimeofday(&tv, NULL);
  return true;
}

// ---------- SAADA CANSATILE: TIME ----------
void sendTimeToCanSat() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  int64_t ms = (int64_t)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);

  char msg[32];
  snprintf(msg, sizeof(msg), "TIME,%lld", ms);

  esp_now_send(CANSAT_MAC, (uint8_t*)msg, strlen(msg));
  Serial.println("[GS] TIME sent -> CanSat");
}

// ---------- SAADA CANSATILE: START ----------
void sendStartToCanSat() {
  const char *msg = "START";
  esp_now_send(CANSAT_MAC, (uint8_t*)msg, strlen(msg));
  Serial.println("[GS] START sent");
}

// ---------- TELEMEETRIA VASTUVÕTT ----------
void onTelemetry(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  // Logi SD-le
  if (sd_ok && logFile) {
    logFile.write(data, len);
  }

  // Serial pass-through (täpselt sama rida, ilma muutmata!)
  Serial.write(data, len);
}

// ---------- ESP-NOW INIT ----------
void initEspNow() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[GS] ESP-NOW INIT FAIL");
    return;
  }

  esp_now_register_recv_cb(onTelemetry);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CANSAT_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[GS] CANSAT PEER ADD FAIL");
  } else {
    Serial.println("[GS] PEER OK");
  }
}

// ---------- SD INIT ----------
void initSD() {
  spiSD.begin(18, 19, 23, SD_CS);

  if (!SD.begin(SD_CS, spiSD, 25000000)) {
    Serial.println("[GS] SD FAIL");
    sd_ok = false;
    return;
  }

  String fname = "/GSLOG01.csv";
  for (int i = 1; i <= 9999; i++) {
    char temp[20];
    sprintf(temp, "/GSLOG%04d.csv", i);
    if (!SD.exists(temp)) {
      fname = String(temp);
      break;
    }
  }

  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) {
    Serial.println("[GS] SD FILE FAIL");
    sd_ok = false;
  } else {
    sd_ok = true;
    Serial.print("[GS] SD LOG -> ");
    Serial.println(fname);
  }
}

// ---------- SERIAL KÄSUD (MAJAAAM) ----------
void processSerialCmd(String line) {
  if (line.length() == 0) return;

  String cmd = line;
  String args = "";
  int sp = line.indexOf(' ');
  if (sp > 0) {
    cmd = line.substring(0, sp);
    args = line.substring(sp + 1);
  }

  cmd.toUpperCase();

  if (cmd == "TIME") {
    if (setRTC_fromString(args)) {
      Serial.println("[GS] RTC SET OK");
      delay(200);
      sendTimeToCanSat();
      delay(200);
      sendTimeToCanSat(); // saadan topelt - kindlam
    } else {
      Serial.println("[GS] RTC PARSE FAIL");
    }
  }

  else if (cmd == "START") {
    sendStartToCanSat();
  }

  else if (cmd == "SENDTIME") {
    sendTimeToCanSat();
  }

  else if (cmd == "HELP") {
    Serial.println("Käsud:");
    Serial.println("  TIME yyyy-mm-dd hh:mm:ss[.mmm]");
    Serial.println("  START");
    Serial.println("  SENDTIME   - saadab TIME uuesti");
  }

  else {
    Serial.println("Tundmatu käsk. Kasuta HELP.");
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[GS] Ground Station 2026");
  initSD();
  initEspNow();
}

// ---------- LOOP ----------
void loop() {
  // Serial command handler
  String line = readSerialLine();
  if (line.length()) processSerialCmd(line);
}
