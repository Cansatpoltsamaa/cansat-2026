/*
  CanSat 2026 – Realtime + SD logi + TIME + MARK + SRC + NTC diagnostika (Variant A)

  MUUDATUSED:
  • CSV on vaikimisi (SET_OUT TABLE -> tabelivaade).
  • Logirea kokkupanek ilma String'ideta (snprintf + fikseeritud puffer).
  • Sama rida (rowbuf) -> Serial + SD; SD_FLUSH_EVERY = 50 + vTaskDelay(1) pärast flush().
  • Serial TX puffer suurem: Serial.setTxBufferSize(512).
  • NTC: Skeem B (SINU skeem) -> NTC_TOP_ON_NTC=false, ADC pin GPIO32, kalibreeritud pinge (analogReadMilliVolts).

  Riistvara:
    ESP32 + LPS22HB (rõhk) + LSM6DS3/DSO/DSOX/DS6 (IMU) + QMC5883P (magnet) + NTC + SD (VSPI)

  Side:
    • ESP‑NOW (ilma Wi‑Fi/ruuterita)
    • CanSat -> ESP‑NOW -> Majaam; Majaam -> (TIME, START) -> CanSat
    • Majaama STA MAC (siht): 78:42:1C:69:3F:48
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LPS2X.h>   // LPS22HB (Unified Sensor API)
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include "driver/adc.h"
#include "esp_system.h"
#include <esp_mac.h>

// FreeRTOS väike viide (vTaskDelay)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- ESP‑NOW SIDE (lisatud) ----------
#include <esp_now.h>
#include <WiFi.h>
#include <sys/time.h>   // settimeofday/gettimeofday

// MAAJAAMA STA MAC (siht, kellele CanSat saadab telemeetria)
uint8_t GS_MAC[6] = { 0x78, 0x42, 0x1C, 0x69, 0x3F, 0x48 };

// START/TIME lipud (TIME -> RTC sünk)
static volatile bool    WIFI_START_RECEIVED = false;
static volatile int64_t WIFI_TIME_EPOCH_MS  = -1;
static volatile int64_t WIFI_TIME_BASE_MS   = 0;

// -------------------- KONFIG --------------------
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

static uint32_t SAMPLE_PERIOD_MS = 250;  // ~4 Hz
static uint32_t HEADER_EVERY_N   = 40;   // päis iga N rea järel Serialis
static bool     OUTPUT_TABLE      = false; // CSV vaikimisi; vajadusel: SET_OUT TABLE

// SD (VSPI). Muuda CS kui vajad (nt 13/5/4/15 ...).
static const int  SD_CS_PIN          = 13;
static const uint32_t SD_FREQ_HZ     = 25000000; // 25 MHz
static const uint32_t SD_FLUSH_EVERY = 50;       // harvem flush -> stabiilsem voog

// ------------- Temperatuuri allikas -------------
static bool USE_LPS_TEMP = false; // true -> Temp(C) LPS22HB; false -> NTC

// ------------- NTC seaded -------------
/*
 Skeem A:
   3.3V --- NTC ---*--- R_SERIES --- GND
                   |
                  ADC

 Skeem B (SINU SKEEM, ADC32):
   3.3V --- R_SERIES ---*--- NTC --- GND
                        |
                       ADC
*/
static const int   NTC_ADC_PIN = 32;       // GPIO32 / ADC1_CH4
static const bool  NTC_TOP_ON_NTC = false; // Skeem B -> FALSE (väga oluline!)
static const int   ADC_BITS     = 12;      // ESP32: 12-bit
static const int   ADC_MAX      = (1<<ADC_BITS)-1;
static const float VREF         = 3.300f;  // valemi Vref (pinge mõõtmine on kalibreeritud!)
static const float R_SERIES     = 10000.0f;// 10k
static const float R0           = 10000.0f;// 10k @ 25°C
static const float BETA         = 3950.0f;
static const float T0_K         = 298.15f; // 25°C kelvinites
static float       TEMP_OFFSET_C = 0.0f;   // vajaduse korral nihutus

// Magnetiline deklinatsioon (Eesti ~8°)
static const float DECLINATION_DEG = 8.0f;

// -------------------- SEADMED --------------------
Adafruit_LPS22   LPS;   // LPS22HB @ 0x5C

// --- LSM6* otse I2C (IMU) ---
static uint8_t LSM6_ADDR       = 0x6A; // tuvastame (0x6A või 0x6B)
static uint8_t LSM6_WHOAMI     = 0x00; // 0x69=DS3, 0x6C=DSO/DSOX/DS6
static uint8_t REG_CTRL1_XL    = 0x10;
static uint8_t REG_CTRL2_G     = 0x11;
static uint8_t REG_CTRL3_C     = 0x12;
static uint8_t REG_OUTX_L_G    = 0x22; // DS3: 0x18; DSOX/DS6: 0x22
static uint8_t REG_OUTX_L_XL   = 0x28; // mõlemal 0x28

// --- QMC5883P madala taseme draiver (0x2C) ---
#define QMC_ADDR 0x2C
enum {
  QMC_REG_X_L = 0x00, QMC_REG_X_H = 0x01,
  QMC_REG_Y_L = 0x02, QMC_REG_Y_H = 0x03,
  QMC_REG_Z_L = 0x04, QMC_REG_Z_H = 0x05,
  QMC_REG_STATUS = 0x06,
  QMC_REG_CONTROL = 0x09,
  QMC_REG_RESET   = 0x0A,
  QMC_REG_PERIOD  = 0x0B,
};
// CONTROL: OSR=512, RNG=8G, ODR=100Hz, Continuous
static const uint8_t QMC_CONTROL_VALUE =
  (0b00 << 6) | (0b01 << 4) | (0b10 << 2) | (0b01 << 0);

bool lps_ok = false, imu_ok = false, mag_ok = false;

// -------------------- PÜSIMÄLU --------------------
Preferences prefs;  // ESP32 NVS
static const char* NVS_NS  = "cansat";
static const uint32_t NVS_VER = 1;

struct MagCal {
  float ox = 0.0f, oy = 0.0f, oz = 0.0f;  // hard-iron offset
  float sx = 1.0f, sy = 1.0f, sz = 1.0f;  // diagonaalne soft-iron skaala
  float heading_zero_deg = 0.0f;          // ZERO_HEAD nihutus
} magcal;

// ΔP kõrguse nullpunkt (hPa)
float P_start_hPa = NAN;

// -------------------- STICKY NÄIDUD --------------------
template<typename T>
struct Sticky {
  T val;
  bool valid;
  Sticky() : val(0), valid(false) {}
};

static Sticky<float> st_tempC;                            // Temp(C) - NTC või LPS
static Sticky<float> st_press_hPa;                        // LPS22HB (hPa)
static Sticky<float> st_altRel_m;                         // ΔP alusel (m)
static Sticky<float> st_accX, st_accY, st_accZ;           // m/s^2
static Sticky<float> st_gyrX, st_gyrY, st_gyrZ;           // deg/s
static Sticky<float> st_magX_u, st_magY_u, st_magZ_u;     // kalibreeritud ühikud
static Sticky<float> st_head_deg;                         // heading (°)

static uint32_t line_count = 0;

// -------------------- SD MUUTUJAD --------------------
SPIClass spiSD(VSPI);
static bool sd_ok = false;
static File logFile;       // andmelogi: LOGxxxx.csv
static File evtFile;       // markerid:  EVENTSxxxx.csv
static uint32_t sd_lines_written = 0;

static const char* CSV_HEADER =
  "datetime,t_ms,TempC,Press_hPa,AltRel_m,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX_u,MagY_u,MagZ_u,Head_deg\n";
static const char* EVT_HEADER = "datetime,t_ms,label\n";

// -------------------- AJATEMPEL --------------------
static bool rtc_valid = false;

static bool getTimestamp(char* out, size_t n) {
  if (!rtc_valid) return false;
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  struct tm tmv;
  localtime_r(&tv.tv_sec, &tmv);
  int ms = (int)(tv.tv_usec / 1000);
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
  return true;
}

static bool parseAndSetTime(const String& ts) {
  int Y,M,D,h,m,s,ms=0;
  int n = sscanf(ts.c_str(), "%d-%d-%d %d:%d:%d.%d", &Y,&M,&D,&h,&m,&s,&ms);
  if (n < 6) return false;
  struct tm tmv = {};
  tmv.tm_year = Y - 1900; tmv.tm_mon = M - 1; tmv.tm_mday = D;
  tmv.tm_hour = h; tmv.tm_min = m; tmv.tm_sec = s;
  time_t t = mktime(&tmv);
  if (t <= 0) return false;
  struct timeval tv; tv.tv_sec = t; tv.tv_usec = (ms>0? ms:0) * 1000;
  settimeofday(&tv, nullptr);
  rtc_valid = true;
  return true;
}

// -------------------- ABI --------------------
static void printHeader(bool table) {
  if (table) {
    Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("  t(ms) |  Temp(C) | Press(hPa) | AltRel(m) |   AccX  AccY  AccZ (m/s^2) |   GyrX   GyrY   GyrZ (deg/s) |   MagX   MagY   MagZ (u) | Head(deg) ");
    Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------");
  } else {
    Serial.print(CSV_HEADER);
  }
  line_count = 0;
}

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static float wrapAngleDeg(float a) {
  while (a < 0.0f)   a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

static bool calcAltRelFromDeltaP(float p_now_hPa, float p_start_hPa, float& out_alt_m) {
  if (!(p_now_hPa > 0.0f) || !(p_start_hPa > 0.0f) || isnan(p_now_hPa) || isnan(p_start_hPa)) return false;
  out_alt_m = 8434.5f * logf(p_start_hPa / p_now_hPa);
  return isfinite(out_alt_m);
}

// -------------------- I2C helpers --------------------
static bool i2cReadBlock(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t rd = Wire.requestFrom(addr, (int)len);
  if (rd != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i]=Wire.read();
  return true;
}
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return (Wire.endTransmission() == 0);
}
static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
  return i2cReadBlock(addr, reg, &val, 1);
}

// -------------------- LSM6* otse I2C --------------------
static void lsm6SelectRegsByWhoAmI(uint8_t who) {
  REG_OUTX_L_G  = 0x22;           // DSO/DSOX/DS6
  REG_OUTX_L_XL = 0x28;           // mõlemal
  if (who == 0x69) REG_OUTX_L_G = 0x18; // DS3
}
static void lsm6Init() {
  if (i2cProbe(0x6A)) LSM6_ADDR = 0x6A;
  else if (i2cProbe(0x6B)) LSM6_ADDR = 0x6B;
  else { imu_ok = false; return; }
  uint8_t who=0; if (!i2cRead8(LSM6_ADDR, 0x0F, who)) { imu_ok=false; return; }
  LSM6_WHOAMI = who; lsm6SelectRegsByWhoAmI(who);
  i2cWrite8(LSM6_ADDR, REG_CTRL3_C, 0x44); // IF_INC=1, BDU=1
  i2cWrite8(LSM6_ADDR, REG_CTRL1_XL, 0x48); // ODR=104Hz, ±4g
  i2cWrite8(LSM6_ADDR, REG_CTRL2_G,  0x48); // ODR=104Hz, ±500dps
  delay(10); imu_ok = true;
}
static bool lsm6Read(float& ax, float& ay, float& az, float& gx_dps, float& gy_dps, float& gz_dps) {
  if (!imu_ok) return false;
  uint8_t gb[6]; if (!i2cReadBlock(LSM6_ADDR, REG_OUTX_L_G, gb, 6)) return false;
  int16_t gx = (int16_t)((gb[1] << 8) | gb[0]);
  int16_t gy = (int16_t)((gb[3] << 8) | gb[2]);
  int16_t gz = (int16_t)((gb[5] << 8) | gb[4]);
  uint8_t ab[6]; if (!i2cReadBlock(LSM6_ADDR, REG_OUTX_L_XL, ab, 6)) return false;
  int16_t axi = (int16_t)((ab[1] << 8) | ab[0]);
  int16_t ayi = (int16_t)((ab[3] << 8) | ab[2]);
  int16_t azi = (int16_t)((ab[5] << 8) | ab[4]);

  const float GYRO_SENS_dps_per_LSB = 0.0175f; // ±500 dps
  gx_dps = gx * GYRO_SENS_dps_per_LSB;
  gy_dps = gy * GYRO_SENS_dps_per_LSB;
  gz_dps = gz * GYRO_SENS_dps_per_LSB;

  const float ACC_SENS_g_per_LSB = 0.000122f; // ±4 g
  const float g_to_ms2 = 9.80665f;
  ax = axi * ACC_SENS_g_per_LSB * g_to_ms2;
  ay = ayi * ACC_SENS_g_per_LSB * g_to_ms2;
  az = azi * ACC_SENS_g_per_LSB * g_to_ms2;

  return isfinite(ax)&&isfinite(ay)&&isfinite(az)&&isfinite(gx_dps)&&isfinite(gy_dps)&&isfinite(gz_dps);
}

// -------------------- QMC5883P --------------------
static void qmcInit() {
  i2cWrite8(QMC_ADDR, QMC_REG_RESET,  0x01); delay(10);
  i2cWrite8(QMC_ADDR, QMC_REG_PERIOD, 0x01); delay(5);
  i2cWrite8(QMC_ADDR, QMC_REG_CONTROL,QMC_CONTROL_VALUE); delay(5);
}
static bool qmcReadRaw(int16_t& rx, int16_t& ry, int16_t& rz) {
  uint8_t st = 0;
  if (i2cReadBlock(QMC_ADDR, QMC_REG_STATUS, &st, 1)) {
    if (st & 0x02) { qmcInit(); delay(2); return false; } // overflow
  }
  uint8_t b[6];
  if (!i2cReadBlock(QMC_ADDR, QMC_REG_X_L, b, 6)) return false;
  rx = (int16_t)((b[1] << 8) | b[0]);
  ry = (int16_t)((b[3] << 8) | b[2]);
  rz = (int16_t)((b[5] << 8) | b[4]);
  return true;
}
static float computeHeadingDeg_XY(float mx, float my) {
  float hdg = atan2f(my, mx) * 180.0f / PI;
  hdg += DECLINATION_DEG;
  return wrapAngleDeg(hdg);
}
static bool readMAG_calibrated(float& mx_c, float& my_c, float& mz_c, float& heading_deg) {
  static bool  have_last = false;
  static float last_mx = 0, last_my = 0, last_mz = 0, last_hdg = 0;
  if (!mag_ok) return false;
  int16_t rx, ry, rz; bool ok = qmcReadRaw(rx, ry, rz);
  if (ok) {
    mx_c = (float(rx) - magcal.ox) * magcal.sx;
    my_c = (float(ry) - magcal.oy) * magcal.sy;
    mz_c = (float(rz) - magcal.oz) * magcal.sz;
    float hdg_raw = computeHeadingDeg_XY(mx_c, my_c);
    heading_deg = wrapAngleDeg(hdg_raw - magcal.heading_zero_deg);
    last_mx=mx_c; last_my=my_c; last_mz=mz_c; last_hdg=heading_deg; have_last=true;
    return true;
  }
  if (have_last) { mx_c=last_mx; my_c=last_my; mz_c=last_mz; heading_deg=last_hdg; return true; }
  return false;
}
static bool readMAG_calibrated_nozero(float& mx_c, float& my_c, float& mz_c, float& heading_raw_deg) {
  if (!mag_ok) return false;
  int16_t rx, ry, rz; if (!qmcReadRaw(rx, ry, rz)) return false;
  mx_c = (float(rx) - magcal.ox) * magcal.sx;
  my_c = (float(ry) - magcal.oy) * magcal.sy;
  mz_c = (float(rz) - magcal.oz) * magcal.sz;
  heading_raw_deg = computeHeadingDeg_XY(mx_c, my_c);
  return isfinite(mx_c) && isfinite(my_c) && isfinite(mz_c) && isfinite(heading_raw_deg);
}

// -------------------- NVS --------------------
static void nvsLoad() {
  prefs.begin(NVS_NS, false);
  uint32_t ver = prefs.getUInt("ver", 0);
  if (ver == NVS_VER) {
    magcal.ox = prefs.getFloat("mag_ox", magcal.ox);
    magcal.oy = prefs.getFloat("mag_oy", magcal.oy);
    magcal.oz = prefs.getFloat("mag_oz", magcal.oz);
    magcal.sx = prefs.getFloat("mag_sx", magcal.sx);
    magcal.sy = prefs.getFloat("mag_sy", magcal.sy);
    magcal.sz = prefs.getFloat("mag_sz", magcal.sz);
    magcal.heading_zero_deg = prefs.getFloat("mag_h0", magcal.heading_zero_deg);
  } else {
    prefs.putUInt("ver", NVS_VER);
  }
}
static void nvsSaveMag() {
  prefs.putUInt("ver", NVS_VER);
  prefs.putFloat("mag_ox", magcal.ox);
  prefs.putFloat("mag_oy", magcal.oy);
  prefs.putFloat("mag_oz", magcal.oz);
  prefs.putFloat("mag_sx", magcal.sx);
  prefs.putFloat("mag_sy", magcal.sy);
  prefs.putFloat("mag_sz", magcal.sz);
  prefs.putFloat("mag_h0", magcal.heading_zero_deg);
}
static void nvsFactoryReset() {
  prefs.clear();
  magcal = MagCal{};
  P_start_hPa = NAN;
}

// -------------------- NTC & LPS22 --------------------

// Mediaan (9 proovi) ilma dünaamilise mäluta
static int median9_int(int a[9]) {
  // lihtne bubble-sort 9 elemendile
  for (int i=0;i<8;i++) {
    for (int j=0;j<8-i;j++) {
      if (a[j] > a[j+1]) { int t=a[j]; a[j]=a[j+1]; a[j+1]=t; }
    }
  }
  return a[4];
}

static int readMedianRawNTC() {
  int arr[9];
  for (int i=0;i<9;i++) {
    arr[i] = analogRead(NTC_ADC_PIN);
    delayMicroseconds(250);
  }
  return median9_int(arr);
}

static int readMedianMilliVoltsNTC() {
  int arr[9];
  for (int i=0;i<9;i++) {
    arr[i] = analogReadMilliVolts(NTC_ADC_PIN); // eFuse kalibreeritud pinge mV-s
    delayMicroseconds(250);
  }
  return median9_int(arr);
}

static bool readNTC_core(float& outTempC, int& raw_out, float& v_out, float& r_out) {
  raw_out = readMedianRawNTC();
  int mv   = readMedianMilliVoltsNTC();
  if (raw_out <= 0 || mv <= 0) return false;

  float v = mv / 1000.0f;
  v_out = v;

  if (v < 0.02f || v > (VREF - 0.02f)) return false;

  float rntc;
  if (NTC_TOP_ON_NTC) {
    if (v <= 0.0f) return false;
    rntc = (R_SERIES * (VREF / v)) - R_SERIES;
  } else {
    if ((VREF - v) <= 0.0f) return false;
    rntc = R_SERIES * (v / (VREF - v));
  }
  if (!(rntc > 0.0f) || !isfinite(rntc)) return false;
  r_out = rntc;

  float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(rntc / R0);
  float T_K  = 1.0f / invT;
  outTempC   = (T_K - 273.15f) + TEMP_OFFSET_C;
  return isfinite(outTempC);
}

static bool readNTC(float& outTempC) {
  int raw; float v, r;
  return readNTC_core(outTempC, raw, v, r);
}

static bool readLPSTemp_C(float& outC) {
  if (!lps_ok) return false;
  sensors_event_t pressure_event, temp_event;
  if (!LPS.getEvent(&pressure_event, &temp_event)) return false;
  float tc = temp_event.temperature; // °C
  if (!isfinite(tc)) return false;
  outC = tc;
  return true;
}

static bool readPressure_hPa(float& out_hPa) {
  if (!lps_ok) return false;
  sensors_event_t pressure_event, temp_event;
  if (!LPS.getEvent(&pressure_event, &temp_event)) return false;
  float p = pressure_event.pressure; // hPa
  if (!isfinite(p) || p <= 0.0f) return false;
  out_hPa = p;
  return true;
}

// -------------------- SERIAL KÄSUD --------------------
static String readLineFromSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { String line = buf; buf = ""; return line; }
    buf += c;
  }
  return String();
}
static void toUpperInPlace(String& s) {
  for (size_t i = 0; i < s.length(); ++i) s.setCharAt(i, toupper((unsigned char)s[i]));
}

static void cmdHELP() {
  Serial.println("Käsud:");
  Serial.println("  HELP");
  Serial.println("  START_ALT                -> seab kõrguse nulli (P_start = hetkerõhk)");
  Serial.println("  ZERO_HEAD                -> seab praeguse pea- suuna 0 deg (mag_h0)");
  Serial.println("  MAG_SET_OFFSET ox oy oz  -> seab magneti offsetid (float)");
  Serial.println("  MAG_SET_SCALE  sx sy sz  -> seab magneti skaalad (float)");
  Serial.println("  MAG_SHOW                 -> näita magnetikalibreeringut ja viimaseid lugemeid");
  Serial.println("  SET_OUT TABLE|CSV        -> Serial väljundiformaat (vaikimisi CSV)");
  Serial.println("  SET_RATE <ms>            -> proovi periood millisekundites (20..5000)");
  Serial.println("  SET_HEADER_N <n>         -> päis iga n rea järel (1..1000)");
  Serial.println("  FACTORY_RESET            -> kustutab NVS (tehase sätted)");
  Serial.println("  TIME YYYY-MM-DD HH:MM:SS[.mmm] -> seab kella arvuti ajaga");
  Serial.println("  MARK <tekst>             -> salvestab sündmuse EVENTS CSV-sse");
  Serial.println("  BEGIN_SRC <nimi>         -> alustab koodi salvestust SD-le (/CODE/<nimi>)");
  Serial.println("  SRC <üks_koodi_rida>     -> lisab ühe koodirea");
  Serial.println("  END_SRC                  -> lõpetab koodi salvestuse");
  Serial.println("  TEMP_SRC NTC|LPS         -> vali, kummast sensorist Temp(C) tuleb");
  Serial.println("  NTC_DEBUG                -> prindi NTC toorandmed (ADC, V, R, Temp) ühe korra");
}

static void cmdSTART_ALT() {
  float p;
  if (readPressure_hPa(p)) { P_start_hPa = p; Serial.printf("START_ALT: P_start_hPa = %.2f hPa\n", P_start_hPa); }
  else Serial.println("START_ALT: Rõhulugem ebaõnnestus (LPS22HB?)");
}
static void cmdZERO_HEAD() {
  float mx,my,mz,hdg_raw;
  if (readMAG_calibrated_nozero(mx,my,mz,hdg_raw)) {
    magcal.heading_zero_deg = hdg_raw; nvsSaveMag();
    Serial.printf("ZERO_HEAD: mag_h0 = %.2f deg\n", magcal.heading_zero_deg);
  } else Serial.println("ZERO_HEAD: Magneti lugem ebaõnnestus");
}
static void cmdMAG_SET_OFFSET(const String& args) {
  float ox, oy, oz;
  if (sscanf(args.c_str(), "%f %f %f", &ox, &oy, &oz) == 3) {
    magcal.ox = ox; magcal.oy = oy; magcal.oz = oz; nvsSaveMag();
    Serial.printf("MAG_SET_OFFSET: ox=%.3f oy=%.3f oz=%.3f\n", ox, oy, oz);
  } else Serial.println("Viga: kasuta: MAG_SET_OFFSET ox oy oz");
}
static void cmdMAG_SET_SCALE(const String& args) {
  float sx, sy, sz;
  if (sscanf(args.c_str(), "%f %f %f", &sx, &sy, &sz) == 3) {
    if (sx == 0.0f || sy == 0.0f || sz == 0.0f) { Serial.println("Viga: skaala ei tohi olla 0"); return; }
    magcal.sx = sx; magcal.sy = sy; magcal.sz = sz; nvsSaveMag();
    Serial.printf("MAG_SET_SCALE: sx=%.6f sy=%.6f sz=%.6f\n", sx, sy, sz);
  } else Serial.println("Viga: kasuta: MAG_SET_SCALE sx sy sz");
}
static void cmdMAG_SHOW() {
  Serial.printf("MAG CAL: offset(ox,oy,oz) = (%.3f, %.3f, %.3f)\n", magcal.ox, magcal.oy, magcal.oz);
  Serial.printf("         scale (sx,sy,sz) = (%.6f, %.6f, %.6f)\n", magcal.sx, magcal.sy, magcal.sz);
  Serial.printf("         heading_zero_deg = %.2f\n", magcal.heading_zero_deg);
  float mx,my,mz,hdg;
  if (readMAG_calibrated(mx,my,mz,hdg)) Serial.printf("MAG NOW (cal): X=%.2f Y=%.2f Z=%.2f  Head=%.2f deg\n", mx,my,mz,hdg);
  else Serial.println("MAG NOW: lugemine ebaõnnestus");
}
static void cmdTEMP_SRC(const String& argsU) {
  if (argsU == "NTC") { USE_LPS_TEMP = false; Serial.println("TEMP_SRC: NTC"); }
  else if (argsU == "LPS") { USE_LPS_TEMP = true; Serial.println("TEMP_SRC: LPS22HB"); }
  else Serial.println("Viga: TEMP_SRC NTC|LPS");
}
static void cmdNTC_DEBUG() {
  int raw; float v, r, t;
  bool ok = readNTC_core(t, raw, v, r);
  Serial.println("NTC_DEBUG:");
  Serial.printf("  raw=%d  V=%.4f V  Rntc=%.1f ohm  Temp=%.2f C  (atten=11dB, pin=%d, skeemB=%d)\n",
                raw, v, r, ok?t:NAN, NTC_ADC_PIN, (int)!NTC_TOP_ON_NTC);
  Serial.println("  Test: puuduta sõrmega -> Temp tõuseb; külmakott -> Temp langeb (skeem B: V tõuseb).");
  Serial.println("  Kui ei muutu, kontrolli pinni (GPIO32?) ja skeemi orienteeringut (NTC_TOP_ON_NTC=false).");
}

// ---- MARK & TIME & koodi vastuvõtt ----
static void logMark(const String& label) {
  char ts[32]; bool ht = getTimestamp(ts, sizeof ts);
  if (sd_ok && evtFile) {
    evtFile.printf("%s,%lu,%s\n", ht?ts:"", (unsigned long)millis(), label.c_str());
    evtFile.flush();
  }
  Serial.printf("MARK: %s | %s\n", ht?ts:"(aeg määramata)", label.c_str());
}
static void cmdTIME(const String& ts) {
  if (parseAndSetTime(ts)) {
    char buf[32]; getTimestamp(buf, sizeof buf);
    Serial.printf("TIME: RTC seatud -> %s\n", buf);
  } else {
    Serial.println("TIME: parsimise viga. Kasuta: TIME YYYY-MM-DD HH:MM:SS[.mmm]");
  }
}

// ---- Koodi salvestus SD-le ----
static File srcFile;
static bool srcOpen = false;

static void cmdBEGIN_SRC(const String& name) {
  if (!sd_ok) { Serial.println("BEGIN_SRC: SD ei ole OK"); return; }
  String clean = name; clean.trim();
  if (clean.length()==0) { Serial.println("BEGIN_SRC: failinimi puudu"); return; }
  if (!clean.startsWith("/")) clean = "/CODE/" + clean;
  else if (!clean.startsWith("/CODE/")) clean = "/CODE" + clean; // kindlusta /CODE/
  if (!SD.exists("/CODE")) SD.mkdir("/CODE");
  srcFile = SD.open(clean, FILE_WRITE);
  if (!srcFile) { Serial.println("BEGIN_SRC: ei saa avada"); return; }
  srcOpen = true;
  Serial.print("BEGIN_SRC: OK -> "); Serial.println(clean);
}
static void cmdSRC(const String& line) {
  if (!srcOpen || !srcFile) { Serial.println("SRC: fail pole avatud"); return; }
  srcFile.println(line);
}
static void cmdEND_SRC() {
  if (srcOpen && srcFile) { srcFile.flush(); srcFile.close(); }
  srcOpen = false;
  Serial.println("END_SRC: OK");
}

static void handleSerial() {
  String line = readLineFromSerial();
  if (line.length() == 0) return;

  String cmd = line; cmd.trim();
  String args = "";
  int sp = cmd.indexOf(' ');
  if (sp > 0) { args = cmd.substring(sp + 1); cmd = cmd.substring(0, sp); }
  String cmdU = cmd; toUpperInPlace(cmdU);
  String argsU = args; toUpperInPlace(argsU);

  if (cmdU == "HELP")              cmdHELP();
  else if (cmdU == "START_ALT")    cmdSTART_ALT();
  else if (cmdU == "ZERO_HEAD")    cmdZERO_HEAD();
  else if (cmdU == "MAG_SET_OFFSET") cmdMAG_SET_OFFSET(args);
  else if (cmdU == "MAG_SET_SCALE")  cmdMAG_SET_SCALE(args);
  else if (cmdU == "MAG_SHOW")     cmdMAG_SHOW();
  else if (cmdU == "SET_OUT")      { if (argsU=="TABLE") { OUTPUT_TABLE=true; Serial.println("SET_OUT: TABLE"); } else if (argsU=="CSV") { OUTPUT_TABLE=false; Serial.println("SET_OUT: CSV"); } else Serial.println("Viga: SET_OUT TABLE|CSV"); }
  else if (cmdU == "SET_RATE")     { int ms=args.toInt(); if (ms>=20 && ms<=5000) { SAMPLE_PERIOD_MS=ms; Serial.printf("SET_RATE: %d ms\n", ms);} else Serial.println("Viga: SET_RATE 20..5000"); }
  else if (cmdU == "SET_HEADER_N") { int n=args.toInt(); if (n>=1 && n<=1000) { HEADER_EVERY_N=n; Serial.printf("SET_HEADER_N: %d\n", n);} else Serial.println("Viga: SET_HEADER_N 1..1000"); }
  else if (cmdU == "FACTORY_RESET") { nvsFactoryReset(); Serial.println("FACTORY_RESET: NVS tühjendatud."); }

  else if (cmdU == "TIME")         cmdTIME(args);
  else if (cmdU == "MARK")         logMark(args);
  else if (cmdU == "BEGIN_SRC")    cmdBEGIN_SRC(args);
  else if (cmdU == "SRC")          cmdSRC(args);
  else if (cmdU == "END_SRC")      cmdEND_SRC();

  else if (cmdU == "TEMP_SRC")     cmdTEMP_SRC(argsU);
  else if (cmdU == "NTC_DEBUG")    cmdNTC_DEBUG();

  else {
    Serial.print("Tundmatu käsk: "); Serial.println(cmd);
    Serial.println("Sisesta HELP");
  }
}

// -------------------- SD FUNKTSIOONID --------------------
static String sdNextFilename(const char* prefix, const char* ext) {
  for (int i=1; i<=9999; ++i) {
    char name[20];
    snprintf(name, sizeof(name), "/%s%04d%s", prefix, i, ext);
    if (!SD.exists(name)) return String(name);
  }
  char name[20]; snprintf(name, sizeof(name), "/%s9999%s", prefix, ext);
  return String(name);
}
static bool sdBegin() {
  spiSD.begin(18, 19, 23, SD_CS_PIN); // SCK, MISO, MOSI, SS
  if (!SD.begin(SD_CS_PIN, spiSD, SD_FREQ_HZ)) return false;
  return true;
}
static bool sdOpenLogPair() {
  String logPath = sdNextFilename("LOG", ".csv");
  String evtPath = logPath; evtPath.replace("LOG", "EVENTS");

  logFile = SD.open(logPath, FILE_WRITE);
  evtFile = SD.open(evtPath, FILE_WRITE);
  if (!logFile || !evtFile) return false;

  logFile.print(CSV_HEADER); logFile.flush();
  evtFile.print(EVT_HEADER); evtFile.flush();
  sd_lines_written = 0;

  Serial.print("SD: LOG fail -> "); Serial.println(logPath);
  Serial.print("SD: EVT fail -> "); Serial.println(evtPath);
  return true;
}
static void writeLogInfo() {
  File f = SD.open("/LOG_INFO.TXT", FILE_APPEND);
  if (!f) return;
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  f.println("CanSat 2026 – Realtime");
  f.printf("Build: %s %s\n", __DATE__, __TIME__);
  f.printf("ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  f.printf("I2C SDA=%d SCL=%d, SD CS=%d\n", I2C_SDA, I2C_SCL, SD_CS_PIN);
  f.printf("SAMPLE_PERIOD_MS=%lu HEADER_EVERY_N=%lu OUTPUT_TABLE=%d\n",
           (unsigned long)SAMPLE_PERIOD_MS, (unsigned long)HEADER_EVERY_N, (int)OUTPUT_TABLE);
  f.printf("TEMP_SRC=%s  NTC_PIN=%d  TOP_ON_NTC=%d  VREF=%.3f  R_SERIES=%.1f  R0=%.1f  BETA=%.0f\n",
           USE_LPS_TEMP?"LPS":"NTC", NTC_ADC_PIN, (int)NTC_TOP_ON_NTC, VREF, R_SERIES, R0, BETA);
  f.printf("MagCal: ox=%.3f oy=%.3f oz=%.3f  sx=%.6f sy=%.6f sz=%.6f  h0=%.2f\n",
           magcal.ox,magcal.oy,magcal.oz,magcal.sx,magcal.sy,magcal.sz,magcal.heading_zero_deg);
  f.close();
}

// -------------------- CSV rea ehitus (ilma String'ideta) --------------------

// Väike abifunktsioon turvaliseks lisamiseks puhvri lõppu
static inline void appendf(char* out, size_t out_sz, int& n, const char* fmt, ...) {
  if (n < 0 || (size_t)n >= out_sz) return;
  va_list ap; va_start(ap, fmt);
  int w = vsnprintf(out + n, out_sz - (size_t)n, fmt, ap);
  va_end(ap);
  if (w > 0) n += w;
}

// Tagastab rea tegeliku pikkuse
static size_t buildCsvRow(char* out, size_t out_sz, uint32_t t_ms) {
  int n = 0;
  char ts[32]; bool ht = getTimestamp(ts, sizeof ts);
  if (ht) appendf(out, out_sz, n, "%s", ts);
  appendf(out, out_sz, n, ",");
  appendf(out, out_sz, n, "%lu,", (unsigned long)t_ms);

  (st_tempC.valid    ) ? appendf(out, out_sz, n, "%.2f,", st_tempC.val)     : appendf(out, out_sz, n, ",");
  (st_press_hPa.valid) ? appendf(out, out_sz, n, "%.2f,", st_press_hPa.val) : appendf(out, out_sz, n, ",");
  (st_altRel_m.valid ) ? appendf(out, out_sz, n, "%.2f,", st_altRel_m.val)  : appendf(out, out_sz, n, ",");

  (st_accX.valid) ? appendf(out, out_sz, n, "%.3f,", st_accX.val) : appendf(out, out_sz, n, ",");
  (st_accY.valid) ? appendf(out, out_sz, n, "%.3f,", st_accY.val) : appendf(out, out_sz, n, ",");
  (st_accZ.valid) ? appendf(out, out_sz, n, "%.3f,", st_accZ.val) : appendf(out, out_sz, n, ",");

  (st_gyrX.valid) ? appendf(out, out_sz, n, "%.3f,", st_gyrX.val) : appendf(out, out_sz, n, ",");
  (st_gyrY.valid) ? appendf(out, out_sz, n, "%.3f,", st_gyrY.val) : appendf(out, out_sz, n, ",");
  (st_gyrZ.valid) ? appendf(out, out_sz, n, "%.3f,", st_gyrZ.val) : appendf(out, out_sz, n, ",");

  (st_magX_u.valid) ? appendf(out, out_sz, n, "%.2f,", st_magX_u.val) : appendf(out, out_sz, n, ",");
  (st_magY_u.valid) ? appendf(out, out_sz, n, "%.2f,", st_magY_u.val) : appendf(out, out_sz, n, ",");
  (st_magZ_u.valid) ? appendf(out, out_sz, n, "%.2f,", st_magZ_u.val) : appendf(out, out_sz, n, ",");

  if (st_head_deg.valid) appendf(out, out_sz, n, "%.2f", st_head_deg.val);
  appendf(out, out_sz, n, "\n");

  if (n < 0) n = (int)out_sz - 1;
  out[out_sz - 1] = '\0';
  return (size_t)n;
}

static void printRow(uint32_t t_ms) {
  if (line_count == 0) printHeader(OUTPUT_TABLE);
  line_count++;

  // Ehita CSV rida KORRA ja taaskasuta
  static char rowbuf[320];
  size_t rlen = buildCsvRow(rowbuf, sizeof(rowbuf), t_ms);

  // Serial
  if (OUTPUT_TABLE) {
    // Tabeliväljund (printf; ei kasuta String'e)
    Serial.printf("%8lu |", (unsigned long)t_ms);
    if (st_tempC.valid)     Serial.printf(" %8.2f |", st_tempC.val);      else Serial.print("   (NA)  |");
    if (st_press_hPa.valid) Serial.printf("   %7.2f |", st_press_hPa.val); else Serial.print("  (NA)   |");
    if (st_altRel_m.valid)  Serial.printf("  %8.2f |", st_altRel_m.val);   else Serial.print("  (NA)   |");

    if (st_accX.valid && st_accY.valid && st_accZ.valid)
      Serial.printf(" %6.2f %6.2f %6.2f |", st_accX.val, st_accY.val, st_accZ.val);
    else Serial.print(" (NA)   (NA)   (NA) |");

    if (st_gyrX.valid && st_gyrY.valid && st_gyrZ.valid)
      Serial.printf(" %7.2f %7.2f %7.2f |", st_gyrX.val, st_gyrY.val, st_gyrZ.val);
    else Serial.print("  (NA)    (NA)    (NA) |");

    if (st_magX_u.valid && st_magY_u.valid && st_magZ_u.valid)
      Serial.printf(" %6.1f %6.1f %6.1f |", st_magX_u.val, st_magY_u.val, st_magZ_u.val);
    else Serial.print("  (NA)   (NA)   (NA) |");

    if (st_head_deg.valid)  Serial.printf("   %7.2f", st_head_deg.val); else Serial.print("   (NA)");
    Serial.println();
  } else {
    // CSV režiimis otse valmis puhvrist
    Serial.write((const uint8_t*)rowbuf, rlen);
  }

  // SD logi (kasuta sama rida)
  if (sd_ok && logFile) {
    logFile.write((const uint8_t*)rowbuf, rlen);
    sd_lines_written++;
    if (sd_lines_written % SD_FLUSH_EVERY == 0) {
      logFile.flush();
      vTaskDelay(1); // väike yield, et vältida pikalt blokeerimist
    }
  }

  // --- ESP‑NOW TELEMEETRIA (lisatud) ---
  esp_now_send(GS_MAC, (uint8_t*)rowbuf, rlen);

  if (line_count >= HEADER_EVERY_N) line_count = 0;
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  Serial.begin(115200);
  // suurenda TX pufferit, et voog stabiilsem oleks
  Serial.setTxBufferSize(512);
  delay(100);
  Serial.println("\nCanSat 2026 – Realtime + SD + TIME + MARK + SRC + NTC diag (Variant A)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(5);

  // NTC ADC sätted: 12-bit, 11 dB (kuni ~3.3 V)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(NTC_ADC_PIN, ADC_11db);
  pinMode(NTC_ADC_PIN, INPUT);

  nvsLoad();

  // SD
  sd_ok = sdBegin();
  if (!sd_ok) Serial.println("SD: EI mountinud (kontrolli juhtmeid/pini ja kaarti)");
  else if (!sdOpenLogPair()) { Serial.println("SD: Ei saanud logifaile avada"); sd_ok = false; }
  else writeLogInfo();

  // LPS22HB
  lps_ok = LPS.begin_I2C(0x5C);
  if (!lps_ok) Serial.println("LPS22HB: EI leitud @0x5C");
  else         Serial.println("LPS22HB: OK");

  // IMU
  lsm6Init();
  if (!imu_ok) Serial.println("LSM6*: EI leitud (0x6A/0x6B)");
  else Serial.printf("LSM6*: OK @0x%02X  WHO_AM_I=0x%02X  GyroOUT=0x%02X  AccOUT=0x%02X\n",
                     LSM6_ADDR, LSM6_WHOAMI, REG_OUTX_L_G, REG_OUTX_L_XL);

  // QMC5883P
  mag_ok = i2cProbe(QMC_ADDR);
  if (!mag_ok) Serial.println("QMC5883P: EI leitud @0x2C");
  else { qmcInit(); Serial.println("QMC5883P: OK (kalibreerimine alati sees, DRDY pinni ei kasutata)"); }

  // --- ESP‑NOW INIT (lisatud) ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {

    // Vastuvõtt START/TIME jaoks
    esp_now_register_recv_cb(onCommand);

    // Registreeri Majaam peer'iks (kuhu saadame telemeetria)
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, GS_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.println("[ESP-NOW] LINK OK");
    } else {
      Serial.println("[ESP-NOW] PEER FAIL");
    }

  } else {
    Serial.println("[ESP-NOW] init FAIL");
  }
}

// ESP‑NOW käsud: START/TIME (lisatud)
void onCommand(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  char buf[64];
  if (len > 63) len = 63;
  memcpy(buf, data, len);
  buf[len] = 0;

  if (strcmp(buf, "START") == 0) {
    WIFI_START_RECEIVED = true;
    Serial.println("[CMD] START received");
    return;
  }

  if (strncmp(buf, "TIME,", 5) == 0) {
    int64_t ms = atoll(buf + 5);
    WIFI_TIME_EPOCH_MS = ms;
    WIFI_TIME_BASE_MS = millis();

    struct timeval tv;
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    settimeofday(&tv, NULL);

    rtc_valid = true;
    Serial.println("[CMD] TIME sync OK");
    return;
  }
}

void loop() {
  static uint32_t t_prev = 0;

  // töötle käsud igal tsüklil
  handleSerial();

  uint32_t t_now = millis();
  if (t_now - t_prev < SAMPLE_PERIOD_MS) return;
  t_prev = t_now;

  // --- Loe andurid ja uuenda "sticky" väärtused ---

  // Temp (NTC või LPS)
  float tempC;
  if ((USE_LPS_TEMP ? readLPSTemp_C(tempC) : readNTC(tempC))) {
    st_tempC.val = tempC; st_tempC.valid = true;
  } else {
    st_tempC.valid = false;
  }

  // Rõhk ja relatiivne kõrgus
  float p_hPa;
  if (readPressure_hPa(p_hPa)) {
    st_press_hPa.val = p_hPa; st_press_hPa.valid = true;
    if (isnan(P_start_hPa)) P_start_hPa = p_hPa; // esimesel edukal lugemisel nullpunkt
    float alt_m;
    if (calcAltRelFromDeltaP(p_hPa, P_start_hPa, alt_m)) {
      st_altRel_m.val = alt_m; st_altRel_m.valid = true;
    } else {
      st_altRel_m.valid = false;
    }
  } else {
    st_press_hPa.valid = false; st_altRel_m.valid = false;
  }

  // IMU
  float ax, ay, az, gx, gy, gz;
  if (lsm6Read(ax, ay, az, gx, gy, gz)) {
    st_accX.val = ax; st_accY.val = ay; st_accZ.val = az;
    st_gyrX.val = gx; st_gyrY.val = gy; st_gyrZ.val = gz;
    st_accX.valid = st_accY.valid = st_accZ.valid = true;
    st_gyrX.valid = st_gyrY.valid = st_gyrZ.valid = true;
  } else {
    st_accX.valid = st_accY.valid = st_accZ.valid = false;
    st_gyrX.valid = st_gyrY.valid = st_gyrZ.valid = false;
  }

  // Magnet + heading (kalibreeritud)
  float mx, my, mz, hdg;
  if (readMAG_calibrated(mx, my, mz, hdg)) {
    st_magX_u.val = mx; st_magY_u.val = my; st_magZ_u.val = mz;
    st_magX_u.valid = st_magY_u.valid = st_magZ_u.valid = true;
    st_head_deg.val = hdg; st_head_deg.valid = true;
  } else {
    st_magX_u.valid = st_magY_u.valid = st_magZ_u.valid = false;
    st_head_deg.valid = false;
  }

  // --- Väljasta/logi üks rida ---
  printRow(t_now);
}