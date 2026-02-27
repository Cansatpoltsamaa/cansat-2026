// ========================================================
// CanSat 2026 – Minimal (variant A)
// - LPS22HB (rõhk -> kõrgus rõhuerinevusest)
// - NTC (temperatuur, Beta)
// - LSM6 (acc + gyro, ilma kalibreerimiseta)
// - QMC5883P (mag X,Y,Z + heading; hard-coded kalibratsioon)
// - SD (CS=4), CSV (timestamp ees), ESP-NOW telemeetria
// - Käsud: TIME, START, START_ALT
// - ESP32 Arduino core 3.3.7 (uued ESP-NOW callbackid)
// ========================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>

#include <time.h>
#include <sys/time.h>

#include <Adafruit_LPS2X.h>  // LPS22HB

// ------------------ Riistvara pinnid --------------------
#define I2C_SDA   21
#define I2C_SCL   22
#define SD_CS     4
#define NTC_ADC   32          // skeem B: 3.3V—R—*—NTC—GND (* = ADC32)

// ------------------ ESP-NOW -----------------------------
static const uint8_t GS_MAC[6] = {0x78,0x42,0x1C,0x69,0x3F,0x48}; // Majaam
static const int WIFI_CH = 1;

// ------------------ Andurid ja olek ---------------------
Adafruit_LPS22 lps;     bool lps_ok=false, imu_ok=false, mag_ok=false, sd_ok=false;
File logFile;

static uint32_t PERIOD_MS = 500;
static bool telemetry_on = false;
static uint32_t t0_ms = 0;

static float P0_hPa = NAN;   // baro nullrõhk (START_ALT)

// ------------------ QMC5883P kalibreerimine -------------
static const float MAG_OX = -8.9f;
static const float MAG_OY = -34.8f;
static const float MAG_OZ = 31.6f;
static const float MAG_SX = 1.0f;
static const float MAG_SY = 1.0f;
static const float MAG_SZ = 1.0f;
static const float MAG_H0 = 0.0f;    // heading nullpunkt
static const float DECLINATION_DEG = 8.0f;  // Eesti ~8°

// ------------------ NTC seaded (Beta) -------------------
static const int   ADC_BITS = 12;
static const float VREF     = 3.300f;
static const float R_SERIES = 10000.0f; // 10k
static const float R0       = 10000.0f; // 10k @25C
static const float BETA     = 3950.0f;
static const float T0_K     = 298.15f;  // 25C
static const float TEMP_OFF = 0.0f;

// --------------------------------------------------------
// ABI: 64-bit millis
// --------------------------------------------------------
static inline uint64_t millis64() { return (uint64_t)(esp_timer_get_time()/1000ULL); }

// --------------------------------------------------------
// ABI: Timestamp (YYYY/MM/DD HH:MM:SS.mmm)
// --------------------------------------------------------
static void formatTimestamp(char* out, size_t n) {
  struct timeval tv; gettimeofday(&tv, nullptr);
  time_t sec = tv.tv_sec;
  int ms = tv.tv_usec/1000;
  struct tm tmv; localtime_r(&sec, &tmv);
  snprintf(out, n, "%04d/%02d/%02d %02d:%02d:%02d.%03d",
           tmv.tm_year+1900, tmv.tm_mon+1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
}

// --------------------------------------------------------
// ABI: parsi TIME "YYYY/MM/DD HH:MM:SS.mmm" -> settimeofday
// --------------------------------------------------------
static bool setRTC_fromString(const char* s) {
  int Y,M,D,h,m,sc,ms=0;
  int c = sscanf(s, "%d/%d/%d %d:%d:%d.%d", &Y,&M,&D,&h,&m,&sc,&ms);
  if (c < 6) return false;
  struct tm tmv = {};
  tmv.tm_year = Y-1900; tmv.tm_mon = M-1; tmv.tm_mday = D;
  tmv.tm_hour = h; tmv.tm_min = m; tmv.tm_sec = sc;
  time_t t = mktime(&tmv);
  if (t <= 0) return false;
  struct timeval tv; tv.tv_sec = t; tv.tv_usec = (ms>0?ms:0)*1000;
  settimeofday(&tv, nullptr);
  return true;
}

// --------------------------------------------------------
// ABI: I2C helperid
// --------------------------------------------------------
static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, int len){
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false)) return false;
  int got = Wire.requestFrom((int)addr, len);
  if (got != len) return false;
  for (int i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
  return (Wire.endTransmission() == 0);
}
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// --------------------------------------------------------
// LSM6 (IMU) – lihtne init ja lugemine
// --------------------------------------------------------
static uint8_t LSM6_ADDR = 0x6A;
static uint8_t REG_OUTX_L_G = 0x22; // DSOX/DS6
static uint8_t REG_OUTX_L_XL= 0x28;

static void imuSelectByWho(uint8_t who) {
  // DS3: gyro OUTX_L 0x18; DSOX/DS6: 0x22
  REG_OUTX_L_G = (who == 0x69) ? 0x18 : 0x22;
  REG_OUTX_L_XL= 0x28;
}

static void imuInit() {
  if      (i2cProbe(0x6A)) LSM6_ADDR = 0x6A;
  else if (i2cProbe(0x6B)) LSM6_ADDR = 0x6B;
  else { imu_ok=false; return; }
  uint8_t who=0; if (!i2cRead(LSM6_ADDR, 0x0F, &who, 1)) { imu_ok=false; return; }
  imuSelectByWho(who);
  // CTRL3_C: IF_INC=1 (0x04), BDU=1 (0x40) -> 0x44
  i2cWrite8(LSM6_ADDR, 0x12, 0x44);
  // CTRL1_XL: ODR=104 Hz, ±4g -> 0x48
  i2cWrite8(LSM6_ADDR, 0x10, 0x48);
  // CTRL2_G : ODR=104 Hz, ±500 dps -> 0x48
  i2cWrite8(LSM6_ADDR, 0x11, 0x48);
  delay(10);
  imu_ok = true;
}

static bool imuRead(float &ax,float &ay,float &az,float &gx,float &gy,float &gz){
  if (!imu_ok) return false;
  uint8_t gb[6], ab[6];
  if (!i2cRead(LSM6_ADDR, REG_OUTX_L_G, gb, 6)) return false;
  if (!i2cRead(LSM6_ADDR, REG_OUTX_L_XL, ab, 6)) return false;

  int16_t gxi = (int16_t)((gb[1]<<8)|gb[0]);
  int16_t gyi = (int16_t)((gb[3]<<8)|gb[2]);
  int16_t gzi = (int16_t)((gb[5]<<8)|gb[4]);

  int16_t axi = (int16_t)((ab[1]<<8)|ab[0]);
  int16_t ayi = (int16_t)((ab[3]<<8)|ab[2]);
  int16_t azi = (int16_t)((ab[5]<<8)|ab[4]);

  gx = gxi * 0.0175f; gy = gyi * 0.0175f; gz = gzi * 0.0175f; // dps
  const float s = 0.000122f * 9.80665f; // ±4g skaalale
  ax = axi * s; ay = ayi * s; az = azi * s;
  return true;
}

// --------------------------------------------------------
// QMC5883P
// --------------------------------------------------------
#define QMC_ADDR 0x2C
static void qmcInit() {
  i2cWrite8(QMC_ADDR, 0x0A, 0x01); delay(5); // RESET
  i2cWrite8(QMC_ADDR, 0x0B, 0x01); delay(5); // PERIOD
  i2cWrite8(QMC_ADDR, 0x09, 0x15); delay(5); // OSR=256,RNG=2G,ODR=50Hz,MODE=CONT
}

static inline float wrap360(float a){ while(a<0)a+=360; while(a>=360)a-=360; return a; }

static bool qmcRead(float &mx,float &my,float &mz,float &hdg){
  uint8_t b[6];
  if (!i2cRead(QMC_ADDR, 0x00, b, 6)) return false;
  int16_t rx=(int16_t)((b[1]<<8)|b[0]);
  int16_t ry=(int16_t)((b[3]<<8)|b[2]);
  int16_t rz=(int16_t)((b[5]<<8)|b[4]);

  mx = ( (float)rx - MAG_OX ) * MAG_SX;
  my = ( (float)ry - MAG_OY ) * MAG_SY;
  mz = ( (float)rz - MAG_OZ ) * MAG_SZ;

  float h = atan2f(my, mx) * 180.0f / PI;
  h += DECLINATION_DEG;
  h -= MAG_H0;
  hdg = wrap360(h);
  return isfinite(mx)&&isfinite(my)&&isfinite(mz)&&isfinite(hdg);
}

// --------------------------------------------------------
// NTC (Beta mudel; skeem B)
// --------------------------------------------------------
static int median9(int a[9]){ for(int i=0;i<8;i++) for(int j=0;j<8-i;j++) if(a[j]>a[j+1]){int t=a[j];a[j]=a[j+1];a[j+1]=t;} return a[4]; }

static bool readNTC(float &tC){
  int rawv[9], mvv[9];
  for(int i=0;i<9;i++){ rawv[i]=analogRead(NTC_ADC); delayMicroseconds(250); }
  for(int i=0;i<9;i++){ mvv[i]=analogReadMilliVolts(NTC_ADC); delayMicroseconds(250); }
  int raw = median9(rawv);
  int mv  = median9(mvv);
  if (raw<=0 || mv<=0) return false;

  float v = mv/1000.0f;
  if (v < 0.02f || v > (VREF-0.02f)) return false;

  // Skeem B: Rntc = R_SERIES * v/(Vref - v)
  float rntc = R_SERIES * (v / (VREF - v));
  if (!(rntc>0.0f) || !isfinite(rntc)) return false;

  float invT = (1.0f/T0_K) + (1.0f/BETA)*logf(rntc/R0);
  float TK = 1.0f/invT;
  tC = (TK - 273.15f) + TEMP_OFF;
  return isfinite(tC);
}

// --------------------------------------------------------
// Baro: rõhk (hPa) ja kõrgus (m) = 8434.5 * ln(P0/P)
// --------------------------------------------------------
static bool readPressure_hPa(float &p_hPa){
  sensors_event_t pe, te;
  if (!lps_ok) return false;
  if (!lps.getEvent(&pe,&te)) return false;
  if (!(pe.pressure > 0.0f)) return false;
  p_hPa = pe.pressure;
  return isfinite(p_hPa);
}
static bool computeAlt_m(float p, float &alt){
  if (isnan(P0_hPa) || !(p>0.0f)) return false;
  alt = 8434.5f * logf(P0_hPa / p);
  return isfinite(alt);
}

// --------------------------------------------------------
// SD: failinimi + header + kirjutus
// --------------------------------------------------------
SPIClass spiSD(VSPI);
static const uint32_t SD_FLUSH_EVERY = 20;
static uint32_t sd_lines = 0;

static String nextLogName(){
  for(int i=1;i<=9999;i++){
    char nm[20]; snprintf(nm,sizeof(nm),"/LOG%04d.csv",i);
    if(!SD.exists(nm)) return String(nm);
  }
  return String("/LOG9999.csv");
}
static bool sdOpenNew(){
  if (logFile) { logFile.flush(); logFile.close(); }
  String path = nextLogName();
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) { Serial.println("SD: open FAIL"); return false; }
  logFile.println("datetime,t_ms,tempC,press_hPa,alt_m,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,heading,magStrength");
  logFile.flush();
  sd_lines = 0;
  Serial.print("SD: "); Serial.println(path);
  return true;
}

// --------------------------------------------------------
// CSV ehitus (turvaline appendifunktsioon)
// --------------------------------------------------------
static void APP(char* out, size_t out_sz, int &n, const char* fmt, ...) {
  if (n < 0 || (size_t)n >= out_sz) return;
  va_list ap; va_start(ap, fmt);
  int w = vsnprintf(out + n, out_sz - (size_t)n, fmt, ap);
  va_end(ap);
  if (w > 0) n += w;
}

static void buildCsvLine(char* out, size_t out_sz,
                         const char* timestr, uint32_t t_ms,
                         float tC, bool t_ok,
                         float p_hPa, bool p_ok,
                         float alt_m, bool alt_ok,
                         float ax,float ay,float az,bool acc_ok,
                         float gx,float gy,float gz,bool gyr_ok,
                         float mx,float my,float mz,float hdg,bool mag_ok_now) {
  int n=0;
  APP(out,out_sz,n,"%s,%lu,", timestr, (unsigned long)t_ms);
  if (t_ok) APP(out,out_sz,n,"%.2f,", tC); else APP(out,out_sz,n,",");
  if (p_ok) APP(out,out_sz,n,"%.2f,", p_hPa); else APP(out,out_sz,n,",");
  if (alt_ok) APP(out,out_sz,n,"%.2f,", alt_m); else APP(out,out_sz,n,",");

  if (acc_ok) APP(out,out_sz,n,"%.3f,%.3f,%.3f,", ax,ay,az); else APP(out,out_sz,n,",,,");

  if (gyr_ok) APP(out,out_sz,n,"%.3f,%.3f,%.3f,", gx,gy,gz); else APP(out,out_sz,n,",,,");

  if (mag_ok_now) {
    float mstr = sqrtf(mx*mx+my*my+mz*mz);
    APP(out,out_sz,n,"%.2f,%.2f,%.2f,%.2f,%.2f", mx,my,mz,hdg,mstr);
  }
  APP(out,out_sz,n,"\n");
}

// --------------------------------------------------------
// ESP-NOW callbackid (ESP32 core 3.3.7+ signatuur)
// --------------------------------------------------------
static void onNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len<=0) return;
  // TIME,YYYY/MM/DD HH:MM:SS.mmm  |  START  |  START_ALT (valikuline GS-lt)
  char buf[96]; int L = min(len, (int)sizeof(buf)-1); memcpy(buf,data,L); buf[L]=0;

  if (!strcmp(buf,"START")) {
    telemetry_on = true; t0_ms = millis();
    Serial.println("[CMD] START");
    return;
  }
  if (!strcmp(buf,"START_ALT")) {
    float p;
    if (readPressure_hPa(p)) { P0_hPa = p; Serial.printf("[CMD] START_ALT: P0=%.2f hPa\n", P0_hPa); }
    return;
  }
  if (!strncmp(buf,"TIME,",5)) {
    const char* ts = buf+5;
    if (setRTC_fromString(ts)) {
      Serial.print("[CMD] TIME: "); Serial.println(ts);
      if (sd_ok) sdOpenNew();
    } else {
      Serial.println("[CMD] TIME: parse FAIL");
    }
    return;
  }
}

static void onNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // soovi korral: Serial.println(status==ESP_NOW_SEND_SUCCESS?"TX OK":"TX FAIL");
}

// --------------------------------------------------------
// Serial käsud: TIME, START, START_ALT (minimaalne)
// --------------------------------------------------------
static String readLine() {
  static String acc;
  while (Serial.available()) {
    char c=(char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') { String s=acc; acc=""; return s; }
    acc += c;
  }
  return String();
}
static void handleSerial() {
  String L = readLine();
  if (L.length()==0) return;
  L.trim();
  String cmd=L, arg="";
  int sp = L.indexOf(' ');
  if (sp>0) { cmd=L.substring(0,sp); arg=L.substring(sp+1); }
  cmd.toUpperCase();

  if (cmd=="START") {
    telemetry_on = true; t0_ms = millis();
    Serial.println("START");
    return;
  }
  if (cmd=="TIME") {
    if (setRTC_fromString(arg.c_str())) {
      Serial.print("TIME: "); Serial.println(arg);
      if (sd_ok) sdOpenNew();
    } else Serial.println("TIME viga. Formaat: YYYY/MM/DD HH:MM:SS[.mmm]");
    return;
  }
  if (cmd=="START_ALT") {
    float p; if (readPressure_hPa(p)) { P0_hPa=p; Serial.printf("START_ALT: P0=%.2f hPa\n", P0_hPa); }
    else Serial.println("START_ALT: rõhk puudub");
    return;
  }
  Serial.println("Käsud: START | TIME YYYY/MM/DD HH:MM:SS.mmm | START_ALT");
}

// --------------------------------------------------------
// SETUP
// --------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nCanSat 2026 – minimal (A)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(NTC_ADC, ADC_11db);
  pinMode(NTC_ADC, INPUT);

  // SD
  spiSD.begin(18,19,23,SD_CS);
  delay(20);
  if (SD.begin(SD_CS, spiSD, 25000000)) { sd_ok=true; sdOpenNew(); }
  else { sd_ok=false; Serial.println("SD: INIT FAIL"); }

  // LPS22HB
  lps_ok = lps.begin_I2C(0x5C);
  Serial.print("LPS22HB: "); Serial.println(lps_ok?"OK":"NO");

  // IMU
  imuInit();
  Serial.print("LSM6: "); Serial.println(imu_ok?"OK":"NO");

  // MAG
  mag_ok = i2cProbe(QMC_ADDR);
  if (mag_ok) { qmcInit(); Serial.println("QMC5883P: OK"); }
  else        { Serial.println("QMC5883P: NO"); }

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onNowRecv);
    esp_now_register_send_cb(onNowSent);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, GS_MAC, 6);
    peer.channel = WIFI_CH;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peer);
    Serial.println("ESP-NOW: OK");
  } else {
    Serial.println("ESP-NOW: INIT FAIL");
  }

  // Ajavöönd Eesti
  setenv("TZ","EET-2EEST,M3.5.0/3,M10.5.0/4",1);
  tzset();

  // Fallback (kui TIME pole saadud): 2000/01/01 + millis
  struct timeval tv; tv.tv_sec = 946684800; tv.tv_usec = 0; settimeofday(&tv,nullptr);

  Serial.println("Valmis. Käsud: START | TIME y/m/d h:m:s.ms | START_ALT");
}

// --------------------------------------------------------
// LOOP
// --------------------------------------------------------
void loop() {
  handleSerial();

  static uint32_t prev=0;
  uint32_t now = millis();
  if (!telemetry_on) return;
  if (now - prev < PERIOD_MS) return;
  prev = now;

  // 1) Andurid
  float tC=NAN;  bool t_ok = readNTC(tC);

  float p=NAN;   bool p_ok = readPressure_hPa(p);
  if (p_ok && isnan(P0_hPa)) P0_hPa = p;   // esimene eduka rõhuga rida -> null
  float alt=NAN; bool alt_ok = p_ok && computeAlt_m(p, alt);

  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0; bool acc_ok=false, gyr_ok=false;
  if (imu_ok) {
    bool ok = imuRead(ax,ay,az,gx,gy,gz);
    acc_ok = ok; gyr_ok = ok;
  }

  float mx=0,my=0,mz=0,hdg=0; bool mag_ok_now=false;
  if (mag_ok) mag_ok_now = qmcRead(mx,my,mz,hdg);

  // 2) Aeg
  char ts[32]; formatTimestamp(ts,sizeof(ts));
  uint32_t t_ms = now - t0_ms;

  // 3) CSV rida
  static char line[256]; line[0]=0;
  buildCsvLine(line,sizeof(line),
               ts,t_ms,
               tC,t_ok,
               p,p_ok,
               alt,alt_ok,
               ax,ay,az,acc_ok,
               gx,gy,gz,gyr_ok,
               mx,my,mz,hdg,mag_ok_now);

  // 4) SD
  if (sd_ok && logFile) {
    size_t wr = logFile.print(line);
    if (wr != strlen(line)) { Serial.println("SD: WRITE FAIL"); sd_ok=false; }
    else if ((++sd_lines % SD_FLUSH_EVERY)==0) logFile.flush();
  }

  // 5) ESP-NOW (sama rida)
  esp_now_send(GS_MAC, (const uint8_t*)line, strlen(line));

  // 6) Soovi korral ka Serial
  Serial.print(line);
}