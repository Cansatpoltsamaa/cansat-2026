#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Siia lisad hiljem päris andurite teegid
// #include <Adafruit_BME280.h>
// #include <Adafruit_LPS2X.h>
// #include <Some_LSM6DS6_Library.h>
// #include <Some_Magnet_Library.h>

namespace cfg {
  constexpr uint8_t SD_CS = 5;            // MUUDA vastavalt plaadile
  constexpr int TERMISTOR_PIN = 34;       // ESP32 ADC pin (näide)

  // Termistori parameetrid (kohanda kui tead):
  constexpr float SERIES_RES_OHM = 10000.0f;
  constexpr float R0_OHM = 10000.0f;
  constexpr float BETA = 3950.0f;
  constexpr float T0_K = 298.15f;

  constexpr uint32_t LOG_FLUSH_MS = 200;
  constexpr uint16_t LOG_BUFFER_ROWS = 64;

  constexpr uint32_t DT_TERM_MS = 500;
  constexpr uint32_t DT_MAG_MS  = 100;
  constexpr uint32_t DT_BARO_MS = 50;
  constexpr uint32_t DT_IMU_MS  = 10;
  constexpr uint32_t DT_TELEM_MS= 200;
}

struct DataRow {
  uint32_t ms = 0;
  float temp_ext_c = NAN; int temp_ext_raw = -1;
  float mag_x = NAN, mag_y = NAN, mag_z = NAN;
  float pressure_hpa = NAN, altitude_m = NAN;
  float acc_x = NAN, acc_y = NAN, acc_z = NAN;
  float gyro_x = NAN, gyro_y = NAN, gyro_z = NAN;
};

File logFile;
String currentFileName;
DataRow ringBuf[cfg::LOG_BUFFER_ROWS];
uint16_t ringCount = 0;
uint32_t lastFlushMs = 0;

float pressureToAltitude(float press_hPa, float p0_hPa = 1013.25f) {
  if (isnan(press_hPa) || press_hPa <= 0) return NAN;
  return 44330.0f * (1.0f - pow(press_hPa / p0_hPa, 0.190295f));
}

float termistorCelsiusFromADC(int adc) {
  if (adc <= 0 || adc >= 4095) return NAN;
  float v_ratio = adc / 4095.0f;
  float r_therm = (cfg::SERIES_RES_OHM * v_ratio) / (1.0f - v_ratio);
  if (r_therm <= 0) return NAN;
  float invT = (1.0f / cfg::T0_K) + (1.0f / cfg::BETA) * log(r_therm / cfg::R0_OHM);
  float T_K = 1.0f / invT;
  return T_K - 273.15f;
}

bool openLogFile() {
  int idx = 1;
  while (true) {
    currentFileName = String("RUN_") + String(idx) + ".csv";
    if (!SD.exists(currentFileName)) break;
    if (++idx > 9999) return false;
  }
  logFile = SD.open(currentFileName, FILE_WRITE);
  if (!logFile) return false;
  logFile.println(F("ms,temp_ext_c,temp_ext_raw,mag_x,mag_y,mag_z,pressure_hpa,altitude_m,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z"));
  logFile.flush();
  lastFlushMs = millis();
  return true;
}

void pushRowToBuffer(const DataRow& r) {
  if (ringCount < cfg::LOG_BUFFER_ROWS) ringBuf[ringCount++] = r;
  if (ringCount >= cfg::LOG_BUFFER_ROWS || (millis() - lastFlushMs) >= cfg::LOG_FLUSH_MS) {
    if (logFile) {
      for (uint16_t i = 0; i < ringCount; ++i) {
        const DataRow& d = ringBuf[i];
        logFile.print(d.ms); logFile.print(',');
        logFile.print(d.temp_ext_c); logFile.print(',');
        logFile.print(d.temp_ext_raw); logFile.print(',');
        logFile.print(d.mag_x); logFile.print(',');
        logFile.print(d.mag_y); logFile.print(',');
        logFile.print(d.mag_z); logFile.print(',');
        logFile.print(d.pressure_hpa); logFile.print(',');
        logFile.print(d.altitude_m); logFile.print(',');
        logFile.print(d.acc_x); logFile.print(',');
        logFile.print(d.acc_y); logFile.print(',');
        logFile.print(d.acc_z); logFile.print(',');
        logFile.print(d.gyro_x); logFile.print(',');
        logFile.print(d.gyro_y); logFile.print(',');
        logFile.println(d.gyro_z);
      }
      logFile.flush();
      lastFlushMs = millis();
    }
    ringCount = 0;
  }
}

// Ajastaja
struct Task { uint32_t dt; uint32_t nextAt; };
Task tTERM{cfg::DT_TERM_MS,0}, tMAG{cfg::DT_MAG_MS,0}, tBARO{cfg::DT_BARO_MS,0}, tIMU{cfg::DT_IMU_MS,0}, tTELEM{cfg::DT_TELEM_MS,0};
DataRow row;

// TODO: sensori initsialiseerimised lisad hiljem
bool initSensors() { return true; }

void readTermistor(DataRow& r) {
  int adc = analogRead(cfg::TERMISTOR_PIN);
  r.temp_ext_raw = adc;
  r.temp_ext_c = termistorCelsiusFromADC(adc);
}
void readMagnet(DataRow& r) {
  if (isnan(r.mag_x)) { r.mag_x = 0; r.mag_y = 0; r.mag_z = 0; }
}
void readBaro(DataRow& r) {
  if (isnan(r.pressure_hpa)) r.pressure_hpa = 1013.25f;
  r.altitude_m = pressureToAltitude(r.pressure_hpa);
}
void readIMU(DataRow& r) {
  if (isnan(r.acc_z)) { r.acc_x = 0; r.acc_y = 0; r.acc_z = 1.0f; }
  if (isnan(r.gyro_x)) { r.gyro_x = 0; r.gyro_y = 0; r.gyro_z = 0; }
}

void sendTelemetry(const DataRow& d) {
  Serial.print(F("TELEM,"));
  Serial.print(d.ms); Serial.print(',');
  Serial.print(d.altitude_m); Serial print(',');
  Serial.print(d.temp_ext_c); Serial.print(',');
  Serial.print(d.pressure_hpa); Serial.print(',');
  Serial.print(d.acc_x); Serial.print(',');
  Serial.print(d.acc_y); Serial.print(',');
  Serial.print(d.acc_z); Serial.print(',');
  Serial.print(d.gyro_x); Serial.print(',');
  Serial.print(d.gyro_y); Serial.print(',');
  Serial.println(d.gyro_z);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  if (!SD.begin(cfg::SD_CS)) { Serial.println(F("SD init FAIL – logimine keelatud")); }
  else if (!openLogFile())   { Serial.println(F("Logifaili avamine ebaõnnestus")); }
  if (!initSensors())        { Serial.println(F("Andurite init FAIL")); }
  uint32_t now = millis();
  tTERM.nextAt=tMAG.nextAt=tBARO.nextAt=tIMU.nextAt=tTELEM.nextAt=now;
}

void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - tTERM.nextAt) >= 0) { readTermistor(row); tTERM.nextAt += tTERM.dt; }
  if ((int32_t)(now - tMAG.nextAt)  >= 0) { readMagnet(row);    tMAG.nextAt  += tMAG.dt;  }
  if ((int32_t)(now - tBARO.nextAt) >= 0) { readBaro(row);      tBARO.nextAt += tBARO.dt; }
  if ((int32_t)(now - tIMU.nextAt)  >= 0) { readIMU(row);       t