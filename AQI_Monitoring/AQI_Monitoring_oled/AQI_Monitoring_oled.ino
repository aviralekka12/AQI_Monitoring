// ===== AQI MONITORING SYSTEM - ESP32 WiFi Version =====
// Arduino Mega 2560 - Sensor Hub & Display
// Communicates with ESP32 via Serial2 for WiFi connectivity

#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <DHT.h>
#include <EEPROM.h>
#include <NoDelay.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>  // Hardware Watchdog Timer

// ===== WATCHDOG-SAFE DELAY =====
// Feeds the watchdog in 1-second chunks so long waits don't trigger a reset
void wdtDelay(unsigned long ms) {
  while (ms > 0) {
    unsigned long chunk = min(ms, 1000UL);
    delay(chunk);
    wdt_reset();
    ms -= chunk;
  }
}

#define jiofi_switch 46

// ===== SENSOR PIN DEFINITIONS =====
#define DHT_PIN 4
#define DHT_TYPE DHT22

#define MQ7_PIN A1   // CO sensor (analog reading)
#define MQ7_HEATER_PIN 44 // PWM pin for MQ-7 heater via IRFZ44N MOSFET
#define MQ2_PIN A10  // SO2 sensor
#define MQ131_PIN A8 // O3 sensor
#define MQ137_PIN A6 // NH3 sensor
#define NO2_PIN A4   // Fermion MEMS NO2
#define MQ135_PIN A2 // CO2 sensor
#define VOC_PIN A0   // DFRobot MEMS VOC

// ===== MQ-7 HEATER CYCLING (Datasheet: 60s@5V + 90s@1.5V) =====
enum MQ7Phase { MQ7_HEATING, MQ7_SENSING };
MQ7Phase mq7Phase = MQ7_HEATING;
unsigned long mq7PhaseStart = 0;
const unsigned long MQ7_HEAT_DURATION  = 60000;  // 60s cleaning at 5V
const unsigned long MQ7_SENSE_DURATION = 90000;  // 90s sensing at ~1.5V
float lastValidCO = 0.0;       // Cached CO reading from last sensing phase
bool mq7ReadingReady = false;  // True when a valid reading was taken
const uint8_t MQ7_PWM_HIGH = 255;  // 5.0V (100% duty cycle)
const uint8_t MQ7_PWM_LOW  = 77;   // ~1.5V (30% duty: 77/255 * 5V ≈ 1.51V)

// ===== AUTO-BASELINE CORRECTION (ABC) for MEMS sensors =====
unsigned long abcWindowStart = 0;
const unsigned long ABC_WINDOW = 86400000UL;  // 24 hours in ms
float abcMinNO2  = 99.0;   // Track min smoothed voltage for NO2 (GM-102B)
float abcMinTVOC = 99.0;   // Track min smoothed voltage for VOC (GM-502B)
const float ABC_BLEND = 0.3; // Blend rate: 30% toward new minimum per cycle
bool abcEnabled = true;      // Can be disabled during calibration


// DHT Sensor object
// DHT Sensor object
DHT dht(DHT_PIN, DHT_TYPE);

// ===== CALIBRATION & EEPROM =====
// EEPROM Address Map
#define EEPROM_INTERVAL_ADDR 0     // 4 bytes
#define EEPROM_CALIB_ADDR 10       // Start of calibration data (~150 bytes)
#define EEPROM_SENSOR_CFG_ADDR 200 // Moved to 200 to make room for larger CalibData
#define EEPROM_UNIT_MODE_ADDR 220  // Moved to 220

// Measured Load Resistors (RL) in kOhms
#define RL_MQ7 0.694
#define RL_MQ2 0.794
#define RL_MQ131 0.698
#define RL_MQ137 0.783
#define RL_MQ135 0.793
#define RL_DEFAULT 10.0 // For others if needed

// ===== SENSOR ENABLE/DISABLE =====
// Index: 0=CO, 1=CO2, 2=O3, 3=NH3, 4=NO2, 5=SO2, 6=TVOC, 7=PM10, 8=PM2.5, 9=PM1.0
#define NUM_SENSORS 10

struct CalibData {
  float r0_co;    // MQ7
  float r0_co2;   // MQ135
  float r0_o3;    // MQ131
  float r0_nh3;   // MQ137
  float r0_so2;   // MQ2 (or dedicated)
  float r0_no2;   // Fermion NO2 (Baseline Offset or Sensitivity)
  float r0_tvoc;  // MEMS VOC (Baseline)
  float pm1_offset;  // PM1.0 Offset (server only)
  float pm10_offset; // PM10 Offset
  float pm25_offset; // PM2.5 Offset
  
  // Two-Point Calibration
  float zero[NUM_SENSORS]; // Offset (c)
  float span[NUM_SENSORS]; // Slope (m)
  
  uint8_t initialized; // Magic byte (0xA6)
};

CalibData calib = { 
    10.0, 76.63, 100.0, 50.0, 9.83, 2.5, 0.6, 
    0.0, 0.0, 0.0, // PM Offsets default 0
    {0,0,0,0,0,0,0,0,0,0}, // Zero offsets default 0
    {1,1,1,1,1,1,1,1,1,1}, // Span slopes default 1
    0}; // Default value

// Sensor enable/disable config moved to loop below
bool sensorEnabled[NUM_SENSORS] = {true, true, true, true, true, true, true, true, true, true};

// Sensor name labels (used by health diagnostics and menu)
const char* sensorNames[] = {"CO", "CO2", "O3", "NH3", "NO2", "SO2", "TVOC", "PM10", "PM2.5", "PM1.0"};

// ===== UNIT SYSTEM MODE =====
bool whoUnitMode = false;  // false = EPA (ppm/ppb), true = WHO (μg/m³)

struct SensorConfig {
  bool enabled[NUM_SENSORS];
  uint8_t initialized; // Magic byte (0xB6)
};

void saveSensorConfig() {
  SensorConfig cfg;
  for (int i = 0; i < NUM_SENSORS; i++) cfg.enabled[i] = sensorEnabled[i];
  cfg.initialized = 0xB6;
  EEPROM.put(EEPROM_SENSOR_CFG_ADDR, cfg);
  Serial.println(">>> EEPROM: Sensor config saved.");
}

void loadSensorConfig() {
  SensorConfig cfg;
  EEPROM.get(EEPROM_SENSOR_CFG_ADDR, cfg);
  if (cfg.initialized == 0xB6) {
    for (int i = 0; i < NUM_SENSORS; i++) sensorEnabled[i] = cfg.enabled[i];
    Serial.println(">>> EEPROM: Sensor config loaded.");
  } else {
    Serial.println(F(">>> EEPROM: No sensor config found, all enabled."));
    for (int i = 0; i < NUM_SENSORS; i++) sensorEnabled[i] = true;
  }
}


// ===== ROTARY ENCODER =====
#define ENC_SW 32
#define ENC_DT 34
#define ENC_CLK 36

int lastClk = HIGH;
unsigned long lastButtonPress = 0;
bool menuActive = false;


// Forward declarations
void saveCalib();
void loadCalib();
void esp32_status_update();
void oled_display_update();
void data_send_update();
void clear_area_update();
void centerText(String text, int y, int size = 1);

int page = 1;

// Data send interval tracking
unsigned long currentSendInterval = 60000; // Default 60 seconds

// Timer callbacks
noDelay esp32_status(5000,
                     esp32_status_update); // Check ESP32 status every 5 seconds
noDelay oled_display(1000, oled_display_update);
noDelay data_send(currentSendInterval, data_send_update); // 60 seconds
noDelay clear_area(3000, clear_area_update);

// ESP32 Communication via Serial2 (TX2=Pin16, RX2=Pin17)
#define ESP32_SERIAL Serial2
#define ESP32_BAUD 115200
#define ESP32_RESET_PIN 2 // ESP32 EN pin connected to Mega D2

// PM Sensor via Serial3 (RX3=Pin15, TX3=Pin14)
#define PM_SENSOR_SERIAL Serial3
#define PM_SENSOR_BAUD 9600

// PM Data structure for PMS5003/PMS7003 sensor
struct PMData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};
PMData pmData;

// Reset function declaration
void hardResetESP32();

// Connection status
int wifi_status = 0; // 0 = disconnected, 1 = connected
bool esp32_ready = false;
int esp32_fail_count = 0; // Consecutive failure counter for watchdog
int jiofi_fail_count = 0; // Consecutive JioFi/LTE failure counter
bool jiofi_restarting = false; // Guard flag to prevent nested restarts
#define JIOFI_MAX_RETRIES 3 // Max JioFi restart attempts before giving up
bool otaInProgress = false; // Locks OLED to OTA screen during update

// ===== SENSOR DATA STRUCTURE =====
// Smoothing Variables
const float ALPHA = 0.2; // Smoothing Factor (Lower = Smoother/Slower)
float sm_v_co = 0.0;
float sm_v_co2 = 0.0;
float sm_v_o3 = 0.0;
float sm_v_nh3 = 0.0;
float sm_v_no2 = 0.0;
float sm_v_so2 = 0.0;
float sm_v_tvoc = 0.0;
bool smoothingInitialized = false;

struct SensorData {

  float pm1_0; // PM1.0 (µg/m³)
  float pm2_5; // PM2.5 (µg/m³)
  float pm10;  // PM10 (µg/m³)
  float co;    // Carbon Monoxide (ppm)
  float co2;   // Carbon Dioxide (ppm)
  float o3;    // Ozone (ppb)
  float nh3;   // Ammonia (ppm)
  float no2;   // Nitrogen Dioxide (ppb)
  float so2;   // Sulfur Dioxide (ppb)
  float tvoc;  // Total Volatile Organic Compounds (ppb)

  // Environmental Sensors
  float temperature; // Temperature (°C)
  float humidity;    // Humidity (%RH)
  int ldr;           // Light sensor value (0=Night, 1=Day or analog value)

  // Power Monitoring
  float voltage; // Battery voltage (V) - Range: 10.0V to 14.6V
  float current; // Current draw (A)
  float power;   // Power consumption (W)

  // Air Quality Index
  int aqi;                    // Overall AQI value (0-500)
  char dominant_pollutant[6]; // PM2.5, PM10, O3, CO, NO2, SO2
  char aqi_category[15];      // Good, Moderate, Unhealthy, etc.

  // Device Identification
  char device_id[20]; // Unique device identifier
};

// Global sensor data instance
SensorData sensorData;

#define i2c_Address 0x3c
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    //   QT-PY / XIAO

// U8g2 Instance
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ===== BITMAP ICONS =====

// 'b_bl', 12x8px
const unsigned char b_bl[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00};

// 'b0', 12x8px
const unsigned char b_0[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x01, 0x04, 0x01, 0x08, 0x01, 0x08, 0x01, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b1', 12x8px
const unsigned char b_1[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x03, 0x04, 0x03, 0x08, 0x03, 0x08, 0x03, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b2', 12x8px
const unsigned char b_2[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x07, 0x04, 0x07, 0x08, 0x07, 0x08, 0x07, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b3', 12x8px
const unsigned char b_3[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x0f, 0x04, 0x0f, 0x08, 0x0f, 0x08, 0x0f, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b4', 12x8px
const unsigned char b_4[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x1f, 0x04, 0x1f, 0x08, 0x1f, 0x08, 0x1f, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b5', 12x8px
const unsigned char b_5[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x3f, 0x04, 0x3f, 0x08, 0x3f, 0x08, 0x3f, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b6', 12x8px
const unsigned char b_6[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0x7f, 0x04, 0x7f, 0x08, 0x7f, 0x08, 0x7f, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b7', 12x8px
const unsigned char b_7[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0xff, 0x04, 0xff, 0x08, 0xff, 0x08, 0xff, 0x04, 
0xff, 0x07, 0x00, 0x00};
// 'b8', 12x8px
const unsigned char b_8[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0xff, 0x05, 0xff, 0x09, 0xff, 0x09, 0xff, 0x05, 
0xff, 0x07, 0x00, 0x00};
// 'b9', 12x8px
const unsigned char b_9[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0xff, 0x07, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x07, 
0xff, 0x07, 0x00, 0x00};
// 'b10', 12x8px
const unsigned char b_10[] PROGMEM = {0x00, 0x00, 0xff, 0x07, 0xff, 0x07, 0xff, 0x0f, 0xff, 0x0f, 0xff, 0x07, 
0xff, 0x07, 0x00, 0x00};

// Array of all bitmaps for convenience. (Total bytes used to store images in
// PROGMEM = 352)
const unsigned char *battery_level[12] = {b_bl, b_0, b_1, b_2, b_3, b_4,
                                          b_5,  b_6, b_7, b_8, b_9, b_10};

const unsigned char blank[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 'no signal', 11x8px
const unsigned char epd_bitmap_no_wifi[] PROGMEM = {0x09, 0x02, 0x06, 0x02, 0x86, 0x02, 0x89, 0x02, 0xa0, 0x02, 0xa0, 0x02, 
0xa8, 0x02, 0xaa, 0x02};
// 'signal', 11x8px
const unsigned char epd_bitmap_wifi[] PROGMEM = {0x00, 0x02, 0x00, 0x02, 0x80, 0x02, 0x80, 0x02, 0xa0, 0x02, 0xa0, 0x02, 
0xa8, 0x02, 0xaa, 0x02};

const unsigned char dataicon[] PROGMEM = {0x04, 0x26, 0x27, 0x24, 0x24, 0xe4, 0x64, 0x20};

// 'No card', 11x8px
const unsigned char No_card[] PROGMEM = {0x6f, 0x00, 0x91, 0x07, 0x25, 0x04, 0x99, 0x05, 0x19, 0x04, 0xa5, 0x05, 
0x01, 0x04, 0xff, 0x07};
// 'card', 11x8px
const unsigned char card[] PROGMEM = {0x6f, 0x00, 0x91, 0x07, 0x01, 0x04, 0x81, 0x05, 0x01, 0x04, 0x81, 0x05, 
0x01, 0x04, 0xff, 0x07};

bool sd_status = false; // Global SD status


byte state;



// 'Update Check', 11x8px
const unsigned char myBitmapUpdate_Check[] PROGMEM = {0xfa, 0x00, 0x06, 0x01, 0x0e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x82, 0x03, 
0x04, 0x03, 0xf8, 0x02};
// 'Updated', 11x8px
const unsigned char myBitmapUpdated[] PROGMEM = {0xfa, 0x00, 0x06, 0x01, 0x0e, 0x02, 0x40, 0x00, 0x28, 0x00, 0x92, 0x03, 
0x04, 0x03, 0xf8, 0x02};

// 'Download', 11x8px
const unsigned char myBitmapOTA_udate[] PROGMEM = {0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x68, 0x01, 
0xf0, 0x00, 0x60, 0x00};

int otaIconState = 0; // 0=None, 1=Checking, 2=Downloading, 3=Updated

void printText(const char *text, int x = 0, int y = 0, int size = 1) {
  if (size == 1) display.setFont(u8g2_font_6x10_tf);
  else if (size == 2) display.setFont(u8g2_font_7x14B_tf);
  else display.setFont(u8g2_font_courB18_tf);
  
  display.setDrawColor(1);
  display.setCursor(x, y);
  display.print(text);
  // Note: display.sendBuffer() is manual
}

void printText(String text, int x = 0, int y = 0, int size = 1) {
  printText(text.c_str(), x, y, size);
}

// ===== EEPROM FUNCTIONS =====

// (saveCalib and loadCalib moved to line ~1809)

// Save interval to EEPROM

void saveIntervalToEEPROM(unsigned long interval) {
  EEPROM.put(EEPROM_INTERVAL_ADDR, interval);
  Serial.print(F("Saved interval to EEPROM: "));
  Serial.println(interval);
}

// Load interval from EEPROM
unsigned long loadIntervalFromEEPROM() {
  unsigned long interval;
  EEPROM.get(EEPROM_INTERVAL_ADDR, interval);

  // Validate interval (must be >= 5000ms and reasonable max)
  if (interval < 5000 || interval > 3600000) { // 3600000 = 1 hour
    Serial.println("EEPROM interval invalid or empty, using default");
    return 60000; // Return default
  }

  Serial.print(F("Loaded interval from EEPROM: "));
  Serial.println(interval);
  return interval;
}


// ===== AQI CALCULATION FUNCTIONS (US EPA Standard) =====

// Helper function to calculate AQI from concentration using breakpoint table
int calculateAQI(float concentration, float Clow, float Chigh, int Ilow,
                 int Ihigh) {
  if (concentration <= 0)
    return 0;
  return round(((Ihigh - Ilow) / (Chigh - Clow)) * (concentration - Clow) +
               Ilow);
}

// Calculate PM2.5 sub-index (24-hour average, μg/m³)
int calculatePM25_AQI(float pm25) {
  if (pm25 <= 12.0)
    return calculateAQI(pm25, 0.0, 12.0, 0, 50);
  else if (pm25 <= 35.4)
    return calculateAQI(pm25, 12.1, 35.4, 51, 100);
  else if (pm25 <= 55.4)
    return calculateAQI(pm25, 35.5, 55.4, 101, 150);
  else if (pm25 <= 150.4)
    return calculateAQI(pm25, 55.5, 150.4, 151, 200);
  else if (pm25 <= 250.4)
    return calculateAQI(pm25, 150.5, 250.4, 201, 300);
  else if (pm25 <= 500.4)
    return calculateAQI(pm25, 250.5, 500.4, 301, 500);
  else
    return 500;
}

// Calculate PM10 sub-index (24-hour average, μg/m³)
int calculatePM10_AQI(float pm10) {
  if (pm10 <= 54)
    return calculateAQI(pm10, 0, 54, 0, 50);
  else if (pm10 <= 154)
    return calculateAQI(pm10, 55, 154, 51, 100);
  else if (pm10 <= 254)
    return calculateAQI(pm10, 155, 254, 101, 150);
  else if (pm10 <= 354)
    return calculateAQI(pm10, 255, 354, 151, 200);
  else if (pm10 <= 424)
    return calculateAQI(pm10, 355, 424, 201, 300);
  else if (pm10 <= 604)
    return calculateAQI(pm10, 425, 604, 301, 500);
  else
    return 500;
}

// Calculate O3 sub-index (8-hour average, ppb)
int calculateO3_AQI(float o3) {
  if (o3 <= 54)
    return calculateAQI(o3, 0, 54, 0, 50);
  else if (o3 <= 70)
    return calculateAQI(o3, 55, 70, 51, 100);
  else if (o3 <= 85)
    return calculateAQI(o3, 71, 85, 101, 150);
  else if (o3 <= 105)
    return calculateAQI(o3, 86, 105, 151, 200);
  else if (o3 <= 200)
    return calculateAQI(o3, 106, 200, 201, 300);
  else
    return 500;
}

// Calculate CO sub-index (8-hour average, ppm)
int calculateCO_AQI(float co) {
  if (co <= 4.4)
    return calculateAQI(co, 0.0, 4.4, 0, 50);
  else if (co <= 9.4)
    return calculateAQI(co, 4.5, 9.4, 51, 100);
  else if (co <= 12.4)
    return calculateAQI(co, 9.5, 12.4, 101, 150);
  else if (co <= 15.4)
    return calculateAQI(co, 12.5, 15.4, 151, 200);
  else if (co <= 30.4)
    return calculateAQI(co, 15.5, 30.4, 201, 300);
  else if (co <= 50.4)
    return calculateAQI(co, 30.5, 50.4, 301, 500);
  else
    return 500;
}

// Calculate NO2 sub-index (1-hour average, ppb)
int calculateNO2_AQI(float no2) {
  if (no2 <= 53)
    return calculateAQI(no2, 0, 53, 0, 50);
  else if (no2 <= 100)
    return calculateAQI(no2, 54, 100, 51, 100);
  else if (no2 <= 360)
    return calculateAQI(no2, 101, 360, 101, 150);
  else if (no2 <= 649)
    return calculateAQI(no2, 361, 649, 151, 200);
  else if (no2 <= 1249)
    return calculateAQI(no2, 650, 1249, 201, 300);
  else if (no2 <= 2049)
    return calculateAQI(no2, 1250, 2049, 301, 500);
  else
    return 500;
}

// Calculate SO2 sub-index (1-hour average, ppb)
int calculateSO2_AQI(float so2) {
  if (so2 <= 35)
    return calculateAQI(so2, 0, 35, 0, 50);
  else if (so2 <= 75)
    return calculateAQI(so2, 36, 75, 51, 100);
  else if (so2 <= 185)
    return calculateAQI(so2, 76, 185, 101, 150);
  else if (so2 <= 304)
    return calculateAQI(so2, 186, 304, 151, 200);
  else if (so2 <= 604)
    return calculateAQI(so2, 305, 604, 201, 300);
  else if (so2 <= 1004)
    return calculateAQI(so2, 605, 1004, 301, 500);
  else
    return 500;
}

// Helper function to get AQI category name
const char *getAQICategory(int aqi) {
  if (aqi <= 50)
    return "Good";
  else if (aqi <= 100)
    return "Moderate";
  else if (aqi <= 150)
    return "Unhealthy SG";
  else if (aqi <= 200)
    return "Unhealthy";
  else if (aqi <= 300)
    return "Very Unhealthy";
  else
    return "Hazardous";
}

// Calculate overall AQI (max of all sub-indices) and identify dominant
// pollutant
void calculateOverallAQI() {
  int pm25_aqi = sensorEnabled[8] ? calculatePM25_AQI(sensorData.pm2_5) : 0;
  int pm10_aqi = sensorEnabled[7] ? calculatePM10_AQI(sensorData.pm10) : 0;
  int o3_aqi   = sensorEnabled[2] ? calculateO3_AQI(sensorData.o3) : 0;
  int co_aqi   = sensorEnabled[0] ? calculateCO_AQI(sensorData.co) : 0;
  int no2_aqi  = sensorEnabled[4] ? calculateNO2_AQI(sensorData.no2) : 0;
  int so2_aqi  = sensorEnabled[5] ? calculateSO2_AQI(sensorData.so2) : 0;

  // Find maximum AQI
  sensorData.aqi = pm25_aqi;
  strcpy(sensorData.dominant_pollutant, "PM2.5");

  if (pm10_aqi > sensorData.aqi) {
    sensorData.aqi = pm10_aqi;
    strcpy(sensorData.dominant_pollutant, "PM10");
  }
  if (o3_aqi > sensorData.aqi) {
    sensorData.aqi = o3_aqi;
    strcpy(sensorData.dominant_pollutant, "O3");
  }
  if (co_aqi > sensorData.aqi) {
    sensorData.aqi = co_aqi;
    strcpy(sensorData.dominant_pollutant, "CO");
  }
  if (no2_aqi > sensorData.aqi) {
    sensorData.aqi = no2_aqi;
    strcpy(sensorData.dominant_pollutant, "NO2");
  }
  if (so2_aqi > sensorData.aqi) {
    sensorData.aqi = so2_aqi;
    strcpy(sensorData.dominant_pollutant, "SO2");
  }

  // Set AQI category
  strcpy(sensorData.aqi_category, getAQICategory(sensorData.aqi));
}

// ===== WHO UNIT CONVERSIONS =====
float convertO3ToWHO(float ppb) {
  float tempK = sensorData.temperature + 273.15;
  return (ppb * 48.0 * 12.187) / tempK;
}

float convertNO2ToWHO(float ppb) {
  float tempK = sensorData.temperature + 273.15;
  return (ppb * 46.0 * 12.187) / tempK;
}

float convertSO2ToWHO(float ppb) {
  float tempK = sensorData.temperature + 273.15;
  return (ppb * 64.1 * 12.187) / tempK;
}

float convertCOToWHO(float ppm) {
  float tempK = sensorData.temperature + 273.15;
  return (ppm * 28.0 * 12.187) / tempK;
}

float convertCO2ToWHO(float ppm) {
  float tempK = sensorData.temperature + 273.15;
  return (ppm * 44.0 * 12.187) / tempK;
}

float convertNH3ToWHO(float ppm) {
  float tempK = sensorData.temperature + 273.15;
  return (ppm * 17.0 * 12.187) / tempK;
}

float convertTVOCToWHO(float ppb) {
  float tempK = sensorData.temperature + 273.15;
  return (ppb * 92.0 * 12.187) / tempK;
}

void saveUnitMode() {
  EEPROM.write(EEPROM_UNIT_MODE_ADDR, whoUnitMode ? 1 : 0);
  Serial.print(F(">>> EEPROM: Unit mode saved: "));
  Serial.println(whoUnitMode ? F("WHO (ug/m3)") : F("EPA (ppm/ppb)"));
}

void loadUnitMode() {
  byte val = EEPROM.read(EEPROM_UNIT_MODE_ADDR);
  if (val == 0 || val == 1) {
    whoUnitMode = (val == 1);
    Serial.print(F(">>> EEPROM: Unit mode loaded: "));
    Serial.println(whoUnitMode ? F("WHO (ug/m3)") : F("EPA (ppm/ppb)"));
  } else {
    whoUnitMode = false;
    Serial.println(F(">>> EEPROM: No unit mode found, defaulting to EPA"));
  }
}

// ===== PM SENSOR FUNCTIONS =====

// Read and parse PM sensor data from PMS5003/PMS7003
bool readPMData() {
  // Check if data available
  if (PM_SENSOR_SERIAL.available() < 32)
    return false;

  // Find header byte 0x42
  while (PM_SENSOR_SERIAL.available() > 0 && PM_SENSOR_SERIAL.peek() != 0x42) {
    PM_SENSOR_SERIAL.read(); // Discard non-header bytes
  }

  // Need at least 32 bytes
  if (PM_SENSOR_SERIAL.available() < 32)
    return false;

  uint8_t buffer[32];
  PM_SENSOR_SERIAL.readBytes(buffer, 32);

  // Validate header (0x42 0x4D)
  if (buffer[0] != 0x42 || buffer[1] != 0x4D)
    return false;

  // Verify checksum
  uint16_t calcChecksum = 0;
  for (uint8_t i = 0; i < 30; i++) {
    calcChecksum += buffer[i];
  }
  uint16_t recvChecksum = (buffer[30] << 8) | buffer[31];

  if (calcChecksum != recvChecksum)
    return false;

  // Extract atmospheric environment data
  pmData.pm1_0 = (buffer[10] << 8) | buffer[11];
  pmData.pm2_5 = (buffer[12] << 8) | buffer[13];
  pmData.pm10 = (buffer[14] << 8) | buffer[15];

  return true;
}

// ===== ADVANCED CALIBRATION HELPER =====
float applyTwoPointCalibration(float rawVal, int sensorIdx) {
  // Formula: Corrected = (Raw - Zero) * Span
  float corrected = (rawVal - calib.zero[sensorIdx]) * calib.span[sensorIdx];
  if (corrected < 0) corrected = 0;
  return corrected;
}

// ===== ADVANCED T/H COMPENSATION HELPER =====
// Returns Rs/R0 correction factor K based on Temperature (t) and Humidity (h)
// Rs_corrected = Rs_measured / K
// If K > 1, sensor is less sensitive (high Rs), so we divide to normalize.
float getCorrectionFactor(int sensorIdx, float t, float h) {
  if (isnan(t) || isnan(h)) return 1.0;
  
  // Base values
  float t_ref = 20.0;
  float h_ref = 65.0;
  
  // Correction Factor K
  float K = 1.0;
  
  switch(sensorIdx) {
    case 0: // MQ-7 (CO) - Rs decreases with T (K < 1 at high T)
      // Approx: -0.01 per degC, -0.005 per %RH
      K = 1.0 - 0.01 * (t - t_ref) - 0.003 * (h - h_ref);
      break;
      
    case 1: // MQ-135 (CO2) - Rs increases with T (K > 1 at high T) ? 
      // Datasheet: Rs/R0 decreases with T/H (Similar to others). 
      // Approx: -0.005 per degC, -0.003 per %RH
      K = 1.0 - 0.005 * (t - t_ref) - 0.003 * (h - h_ref);
      break;
      
    case 2: // MQ-131 (O3) - Rs decreases with T/H
      K = 1.0 - 0.008 * (t - t_ref) - 0.004 * (h - h_ref);
      break;
      
    case 3: // MQ-137 (NH3)
      K = 1.0 - 0.005 * (t - t_ref) - 0.003 * (h - h_ref);
      break;
      
    case 5: // MQ-2 (SO2/Smoke)
      K = 1.0 - 0.006 * (t - t_ref) - 0.002 * (h - h_ref);
      break;
      
    default:
      return 1.0;
  }
  
  // Safety Limits for K (0.5 to 1.8)
  if (K < 0.5) K = 0.5;
  if (K > 1.8) K = 1.8;
  
  return K;
}


// ===== GAS SENSOR READING FUNCTIONS =====

// Read MQ-7 CO sensor (ppm)
// Only returns live reading during SENSING phase after stabilization; otherwise cached
float readCO() {
  // During heating phase, return cached value
  if (mq7Phase == MQ7_HEATING) {
    return lastValidCO;
  }

  // During sensing phase, wait for stabilization (first 30s are unstable)
  unsigned long senseElapsed = millis() - mq7PhaseStart;
  if (senseElapsed < 30000) {
    // Sensor still stabilizing after heater change, return cached
    return lastValidCO;
  }

  // Multi-sample with PWM artifact filtering
  // Module shares heater/sensor ground through MOSFET, so during PWM OFF 
  // periods the sensor ground floats → reads ~5V (artifact). We filter those out.
  int validCount = 0;
  float voltageSum = 0.0;
  for (int i = 0; i < 20; i++) {
    int adc = analogRead(MQ7_PIN);
    float v = adc * (5.0 / 1023.0);
    if (v < 3.0) {  // Only keep readings during MOSFET ON (real sensor data)
      voltageSum += v;
      validCount++;
    }
    delayMicroseconds(200);  // Spread across PWM cycle (~2ms per cycle at 490Hz)
  }
  
  if (validCount == 0) {
    Serial.println(F("MQ7 DEBUG: No valid samples (all floating)"));
    return lastValidCO;  // All readings were artifacts
  }
  
  float rawVoltage = voltageSum / validCount;
  
  // Use direct assignment (no smoothing carry-over from heating phase)
  sm_v_co = rawVoltage;
  
  if (sm_v_co < 0.1) sm_v_co = 0.1;

  float Rs = ((5.0 * RL_MQ7) / sm_v_co) - RL_MQ7;
  
  // Apply Advanced T/H Compensation
  float K = getCorrectionFactor(0, sensorData.temperature, sensorData.humidity);
  float Rs_comp = Rs / K; 

  float ratio = Rs_comp / calib.r0_co;

  // MQ-7 datasheet curve: ppm = 10^((log(Rs/R0) - 0.74) / -0.36)
  float ppm = pow(10, ((log10(ratio) - 0.74) / -0.36));

  // Legacy simple compensation removed (replaced by getCorrectionFactor)
  
  // Apply Two-Point Calibration (Zero/Span)
  ppm = applyTwoPointCalibration(ppm, 0);

  Serial.print(F(" ppm="));
  Serial.println(ppm, 2);

  ppm = constrain(ppm, 0, 1000);
  lastValidCO = ppm;  // Cache for heating phase
  mq7ReadingReady = true;
  return ppm;
}

// MQ-7 Heater Cycling State Machine (non-blocking, call from loop)
void updateMQ7Heater() {
  unsigned long now = millis();
  unsigned long elapsed = now - mq7PhaseStart;

  if (mq7Phase == MQ7_HEATING && elapsed >= MQ7_HEAT_DURATION) {
    // Switch to sensing phase (1.5V)
    mq7Phase = MQ7_SENSING;
    mq7PhaseStart = now;
    analogWrite(MQ7_HEATER_PIN, MQ7_PWM_LOW);
    // Reset smoothing - old heating voltage is invalid for sensing
    sm_v_co = analogRead(MQ7_PIN) * (5.0 / 1023.0);
    Serial.println(F("MQ7: Sensing phase (1.5V) - 30s stabilize + 60s reading"));
  }
  else if (mq7Phase == MQ7_SENSING && elapsed >= MQ7_SENSE_DURATION) {
    // Take final reading before switching (most stable point)
    if (sensorEnabled[0]) {
      readCO(); // Caches in lastValidCO
    }
    // Switch to heating phase (5V)
    mq7Phase = MQ7_HEATING;
    mq7PhaseStart = now;
    analogWrite(MQ7_HEATER_PIN, MQ7_PWM_HIGH);
    Serial.print(F("MQ7: Heating phase (5V) - CO cached: "));
    Serial.print(lastValidCO);
    Serial.println(F(" ppm"));
  }
}



// Read MQ-135 for CO2 (ppm)
// NOTE: Requires R0 calibration in clean air (400ppm CO2)
// Read MQ-135 for CO2 (ppm)
// Read MQ-135 for CO2 (ppm)
float readCO2() {
  int rawValue = analogRead(MQ135_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_co2 = rawVoltage;
  sm_v_co2 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_co2);

  if (sm_v_co2 < 0.1) sm_v_co2 = 0.1;

  float Rs = ((5.0 * RL_MQ135) / sm_v_co2) - RL_MQ135;
  
  // Apply Advanced T/H Compensation
  float K = getCorrectionFactor(1, sensorData.temperature, sensorData.humidity);
  float Rs_comp = Rs / K;
  
  float ratio = Rs_comp / calib.r0_co2;

  // MQ135 datasheet: ppm = 116.602 * (ratio ^ -2.769)
  float ppm = 116.6020682 * pow(ratio, -2.769034857);

  // Apply Two-Point Calibration
  ppm = applyTwoPointCalibration(ppm, 1);

  return constrain(ppm, 10, 10000);
}



// Read MQ-131 O3 sensor (ppb)
// Using datasheet curve for low-concentration version
// Read MQ-131 O3 sensor (ppb)
// Read MQ-131 O3 sensor (ppb)
float readO3() {
  int rawValue = analogRead(MQ131_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_o3 = rawVoltage;
  sm_v_o3 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_o3);
  
  if (sm_v_o3 < 0.1) sm_v_o3 = 0.1;

  float Rs = ((5.0 * RL_MQ131) / sm_v_o3) - RL_MQ131;
  
  // Apply Advanced T/H Compensation
  float K = getCorrectionFactor(2, sensorData.temperature, sensorData.humidity);
  float Rs_comp = Rs / K;
  
  float ratio = Rs_comp / calib.r0_o3;

  // ppb = 10^((log(Rs/R0) + 0.92) / -0.44)
  float ppb = pow(10, ((log10(ratio) + 0.92) / -0.44));

  // Apply Two-Point Calibration
  ppb = applyTwoPointCalibration(ppb, 2);

  return constrain(ppb, 0, 1000);
}



// Read MQ-137 NH3 sensor (ppm)
// Using MQ137 datasheet specifications (5-500ppm range)
// Read MQ-137 NH3 sensor (ppm)
// Read MQ-137 NH3 sensor (ppm)
float readNH3() {
  int rawValue = analogRead(MQ137_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_nh3 = rawVoltage;
  sm_v_nh3 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_nh3);
  
  if (sm_v_nh3 < 0.1) sm_v_nh3 = 0.1;

  float Rs = ((5.0 * RL_MQ137) / sm_v_nh3) - RL_MQ137;
  
  // Apply Advanced T/H Compensation
  float K = getCorrectionFactor(3, sensorData.temperature, sensorData.humidity);
  float Rs_comp = Rs / K;
  
  float ratio = Rs_comp / calib.r0_nh3;

  // ppm = 50.0 * pow(ratio, -1.2)
  float ppm = 50.0 * pow(ratio, -1.2);

  // Apply Two-Point Calibration
  ppm = applyTwoPointCalibration(ppm, 3);

  return constrain(ppm, 0, 500);
}



// Read GM-102B MEMS NO2 sensor (ppb)
// Using datasheet specifications
// Read GM-102B MEMS NO2 sensor (ppb)
// Read GM-102B MEMS NO2 sensor (ppb)
float readNO2() {
  int rawValue = analogRead(NO2_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_no2 = rawVoltage;
  sm_v_no2 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_no2);

  // Baseline voltage from calibration
  float baselineVoltage = calib.r0_no2;
  float deltaV = sm_v_no2 - baselineVoltage;
  float ppb = deltaV * 1000.0; // Sensitivity (ppb) (Placeholder sensitivity)
  
  // Apply Two-Point Calibration (Zero/Span helps here significantly)
  ppb = applyTwoPointCalibration(ppb, 4);

  // Clamp - don't show negative if below baseline
  if (ppb < 0) ppb = 0;

  return constrain(ppb, 0, 10.0);
}




// Read MQ-2 for SO2 / Smoke (ppb)
// NOTE: MQ2 is primarily for LPG/smoke, SO2 sensitivity is limited
// Requires R0 calibration in clean air
// Read MQ-2 for SO2 / Smoke (ppb)
// Read MQ-2 for SO2 / Smoke (ppb)
float readSO2() {
  int rawValue = analogRead(MQ2_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_so2 = rawVoltage;
  sm_v_so2 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_so2);

  float Rs = ((5.0 * RL_MQ2) / sm_v_so2) - RL_MQ2;
  
  // Apply Advanced T/H Compensation
  float K = getCorrectionFactor(5, sensorData.temperature, sensorData.humidity);
  float Rs_comp = Rs / K;
  
  float ratio = Rs_comp / calib.r0_so2;

  // MQ2 curve
  float ppb = 50.0 * pow(ratio, -1.4);

  // Apply Two-Point Calibration
  ppb = applyTwoPointCalibration(ppb, 5);

  return constrain(ppb, 0, 1000);
}

// Read MEMS VOC sensor (GM-502B)
// Using datasheet specifications
// Read GM-502B MEMS VOC
// Read MEMS VOC sensor (GM-502B)
// Using datasheet specifications
// Read GM-502B MEMS VOC
float lastValidTVOC = 0.0;

float readTVOC() {
  int rawValue = analogRead(VOC_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  // Use much stronger smoothing for TVOC (often noisy)
  const float ALPHA_TVOC = 0.05; 
  if (!smoothingInitialized) sm_v_tvoc = rawVoltage;
  sm_v_tvoc = (ALPHA_TVOC * rawVoltage) + ((1.0 - ALPHA_TVOC) * sm_v_tvoc);

  float baselineVoltage = calib.r0_tvoc; // Baseline
  
  // Safety: If baseline is too close to max (5.0), the divisor becomes 0 -> Infinity
  if (baselineVoltage > 4.8) baselineVoltage = 4.8;
  
  float deltaV = sm_v_tvoc - baselineVoltage;
  
  // If deltaV is very small noise, clamp to 0
  if (deltaV < 0.05) deltaV = 0.0;
  
  float ppb = (deltaV / (5.0 - baselineVoltage)) * 1000;

  if (ppb < 0) ppb = 0;

  // Apply Two-Point Calibration
  ppb = applyTwoPointCalibration(ppb, 6);

  ppb = constrain(ppb, 0, 2000);
  
  // Outlier Rejection
  float diff = ppb - lastValidTVOC;
  if (diff > 50) ppb = lastValidTVOC + 50;
  else if (diff < -50) ppb = lastValidTVOC - 50;
  
  lastValidTVOC = ppb;
  
  return ppb;
}

// ===== AUTO-BASELINE CORRECTION (ABC) =====
// Tracks minimum smoothed voltage over 24h window.
// Assumes sensor sees clean air at least once per window.
// Only adjusts baseline DOWNWARD (toward cleaner air).
void updateABC() {
  if (!abcEnabled) return;
  if (!smoothingInitialized) return;  // Need valid smoothed values
  
  // Track minimum smoothed voltages
  if (sensorEnabled[4] && sm_v_no2 > 0.05 && sm_v_no2 < abcMinNO2) {
    abcMinNO2 = sm_v_no2;
  }
  if (sensorEnabled[6] && sm_v_tvoc > 0.05 && sm_v_tvoc < abcMinTVOC) {
    abcMinTVOC = sm_v_tvoc;
  }
  
  // Check if 24h window has elapsed
  unsigned long now = millis();
  if (now - abcWindowStart >= ABC_WINDOW) {
    // === Apply corrections ===
    bool changed = false;
    
    // NO2: Only adjust if minimum is LOWER than current baseline
    if (abcMinNO2 < 99.0 && abcMinNO2 < calib.r0_no2) {
      float oldBaseline = calib.r0_no2;
      calib.r0_no2 = calib.r0_no2 * (1.0 - ABC_BLEND) + abcMinNO2 * ABC_BLEND;
      calib.r0_no2 = calib.r0_no2 * (1.0 - ABC_BLEND) + abcMinNO2 * ABC_BLEND;
      Serial.print(F("ABC NO2: baseline "));
      Serial.print(oldBaseline, 3);
      Serial.print(F(" -> "));
      Serial.println(calib.r0_no2, 3);
      changed = true;
    }
    
    // TVOC: Only adjust if minimum is LOWER than current baseline
    if (abcMinTVOC < 99.0 && abcMinTVOC < calib.r0_tvoc) {
      float oldBaseline = calib.r0_tvoc;
      calib.r0_tvoc = calib.r0_tvoc * (1.0 - ABC_BLEND) + abcMinTVOC * ABC_BLEND;
      calib.r0_tvoc = calib.r0_tvoc * (1.0 - ABC_BLEND) + abcMinTVOC * ABC_BLEND;
      Serial.print(F("ABC TVOC: baseline "));
      Serial.print(oldBaseline, 3);
      Serial.print(F(" -> "));
      Serial.println(calib.r0_tvoc, 3);
      changed = true;
    }
    
    // Save to EEPROM if any baseline changed
    if (changed) {
      saveCalib();
      Serial.println(F("ABC: Baselines saved to EEPROM"));
    } else {
      Serial.println(F("ABC: No correction needed this cycle"));
    }
    
    // Reset window
    abcMinNO2 = 99.0;
    abcMinTVOC = 99.0;
    abcWindowStart = now;
  }
}


void initSmoothing() {
  sm_v_co = analogRead(MQ7_PIN)*(5.0/1023.0);
  sm_v_co2 = analogRead(MQ135_PIN)*(5.0/1023.0);
  sm_v_o3 = analogRead(MQ131_PIN)*(5.0/1023.0);
  sm_v_nh3 = analogRead(MQ137_PIN)*(5.0/1023.0);
  sm_v_no2 = analogRead(NO2_PIN)*(5.0/1023.0);
  sm_v_so2 = analogRead(MQ2_PIN)*(5.0/1023.0);
  sm_v_tvoc = analogRead(VOC_PIN)*(5.0/1023.0);
  smoothingInitialized = true;
}



// ===== CROSS-SENSITIVITY COMPENSATION =====
// Adjusts sensorData in place to remove interference.
// Called before AQI calculation.
void compensateCrossSensitivity() {
  if (!calib.initialized) return;
  
  // 1. CO Interference on CO2 (MQ-135)
  // MQ-135 is sensitive to CO. If CO is high, CO2 might read artificially high.
  if (sensorEnabled[1] && sensorEnabled[0]) {
     // Approx: 1ppm CO adds ~2ppm equivalent to MQ135 (very rough, depends on temp)
     // Correction: Refine this based on testing.
     // sensorData.co2 = sensorData.co2 - (sensorData.co * 1.5); 
     // Commented out for now - Needs tuning.
  }
  
  // 2. NO2 Interference on O3 (MQ-131)
  // MQ-131 (O3) often reacts to NO2.
  if (sensorEnabled[2] && sensorEnabled[4]) {
     // If NO2 is high, O3 reads might be affected.
  }
  
  // 3. T/H effects are already handled in getCorrectionFactor().
  
  // Prevent negatives after compensation
  if (sensorData.co2 < 0) sensorData.co2 = 0;
  if (sensorData.o3 < 0) sensorData.o3 = 0;
}



// Function to read all sensors and populate sensorData
void readAllSensors() {
  // Read real PM sensor data (skip if all PM sensors disabled)
  if (sensorEnabled[7] || sensorEnabled[8] || sensorEnabled[9]) {
    if (readPMData()) {
      sensorData.pm1_0 = sensorEnabled[9] ? pmData.pm1_0 : 0;
      
      // Apply calibration offsets
      if (sensorEnabled[8]) {
        float p25 = pmData.pm2_5 + calib.pm25_offset;
        if(p25 < 0) p25 = 0;
        sensorData.pm2_5 = p25;
      } else {
        sensorData.pm2_5 = 0;
      }
      
      if (sensorEnabled[7]) {
        float p10 = pmData.pm10 + calib.pm10_offset;
        if(p10 < 0) p10 = 0;
        sensorData.pm10 = p10;
      } else {
        sensorData.pm10 = 0;
      }
      
      Serial.println(F("--- PM Sensor Data (ug/m3) ---"));
      Serial.print(F("PM 1.0: ")); Serial.println(sensorData.pm1_0);
      Serial.print(F("PM 2.5: ")); Serial.println(sensorData.pm2_5);
      Serial.print(F("PM 10:  ")); Serial.println(sensorData.pm10);
    } else {
      Serial.println(F("PM Sensor: No data available, using last values"));
    }
  } else {
    sensorData.pm1_0 = 0;
    sensorData.pm2_5 = 0;
    sensorData.pm10 = 0;
  }

  // Read gas sensors (skip disabled ones)
  sensorData.co   = sensorEnabled[0] ? readCO()   : 0;
  sensorData.co2  = sensorEnabled[1] ? readCO2()  : 0;
  sensorData.o3   = sensorEnabled[2] ? readO3()   : 0;
  sensorData.nh3  = sensorEnabled[3] ? readNH3()  : 0;
  sensorData.no2  = sensorEnabled[4] ? readNO2()  : 0;
  sensorData.so2  = sensorEnabled[5] ? readSO2()  : 0;
  sensorData.tvoc = sensorEnabled[6] ? readTVOC() : 0;

  // Read DHT22 temperature and humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    sensorData.temperature = t;
    sensorData.humidity = h;
  } else {
    Serial.println(F("DHT22: Read failed, using last values"));
  }

  // Power monitoring - these require specific sensors (INA219, voltage divider,
  // etc.) Set to 0 or implement actual sensor reading when available
  sensorData.voltage = 12.8; // Implement with voltage sensor when available
  sensorData.current = 2.5;  // Implement with current sensor when available
  sensorData.power = 32; // Will be voltage * current when sensors are available

  sensorData.power = 32; // Will be voltage * current when sensors are available

  // Apply Cross-Sensitivity Compensation
  compensateCrossSensitivity();

  // Calculate AQI from pollutant values
  calculateOverallAQI();

  // Print sensor summary
  // Print sensor summary
  Serial.println(F("--- Gas Sensor Readings ---"));
  Serial.print(F("CO: "));
  Serial.print(sensorData.co);
  Serial.println(F(" ppm"));
  Serial.print(F("CO2: "));
  Serial.print(sensorData.co2);
  Serial.println(F(" ppm"));
  Serial.print(F("O3: "));
  Serial.print(sensorData.o3);
  Serial.println(F(" ppb"));
  Serial.print(F("NH3: "));
  Serial.print(sensorData.nh3);
  Serial.println(F(" ppm"));
  Serial.print(F("NO2: "));
  Serial.print(sensorData.no2);
  Serial.println(F(" ppb"));
  Serial.print(F("SO2: "));
  Serial.print(sensorData.so2);
  Serial.println(F(" ppb"));
  Serial.print(F("TVOC: "));
  Serial.print(sensorData.tvoc);
  Serial.println(F(" ppb"));
  Serial.println(F("--- Environmental ---"));
  Serial.print(F("Temp: "));
  Serial.print(sensorData.temperature);
  Serial.println(F(" C"));
  Serial.print(F("Humidity: "));
  Serial.print(sensorData.humidity);
  Serial.println(F(" %"));
  Serial.println(F("--- AQI ---"));
  Serial.print(F("AQI: "));
  Serial.print(sensorData.aqi);
  Serial.print(F(" ("));
  Serial.print(sensorData.aqi_category);
  Serial.print(F(") - "));
  Serial.println(sensorData.dominant_pollutant);
}

// Alias for backward compatibility
void generateTestData() { readAllSensors(); }

// Build JSON payload from sensor data
// Stream JSON payload directly to a Print interface (Serial or Serial2)
// This avoids large String buffer allocation issues
void streamJson(Print& p) {
  p.print("{");

  // Device Identification
  p.print("\"device_id\":\""); p.print(sensorData.device_id); p.print("\",");

  // Air Quality Data (Apply PM1.0 calibration for server only)
  float pm1_calibrated = sensorData.pm1_0 + calib.pm1_offset;
  if(pm1_calibrated < 0) pm1_calibrated = 0;

  if (whoUnitMode) {
    // WHO mode (ug/m3 or mg/m3)
    p.print("\"pm1_0\":"); p.print(pm1_calibrated, 1); p.print(",");
    p.print("\"pm2_5\":"); p.print(sensorData.pm2_5, 1); p.print(",");
    p.print("\"pm10\":"); p.print(sensorData.pm10, 1); p.print(",");
    p.print("\"co\":"); p.print(convertCOToWHO(sensorData.co), 3); p.print(",");
    p.print("\"co2\":"); p.print(convertCO2ToWHO(sensorData.co2), 1); p.print(",");
    p.print("\"o3\":"); p.print(convertO3ToWHO(sensorData.o3), 1); p.print(",");
    p.print("\"nh3\":"); p.print(convertNH3ToWHO(sensorData.nh3), 3); p.print(",");
    p.print("\"no2\":"); p.print(convertNO2ToWHO(sensorData.no2), 1); p.print(",");
    p.print("\"so2\":"); p.print(convertSO2ToWHO(sensorData.so2), 1); p.print(",");
    p.print("\"tvoc\":"); p.print(convertTVOCToWHO(sensorData.tvoc), 1); p.print(",");

  } else {
    // EPA mode (ppm/ppb)
    p.print("\"pm1_0\":"); p.print(pm1_calibrated, 2); p.print(",");
    p.print("\"pm2_5\":"); p.print(sensorData.pm2_5, 2); p.print(",");
    p.print("\"pm10\":"); p.print(sensorData.pm10, 2); p.print(",");
    p.print("\"co\":"); p.print(sensorData.co, 3); p.print(",");
    p.print("\"co2\":"); p.print(sensorData.co2, 2); p.print(",");
    p.print("\"o3\":"); p.print(sensorData.o3, 2); p.print(",");
    p.print("\"nh3\":"); p.print(sensorData.nh3, 3); p.print(",");
    p.print("\"no2\":"); p.print(sensorData.no2, 2); p.print(",");
    p.print("\"so2\":"); p.print(sensorData.so2, 2); p.print(",");
    p.print("\"tvoc\":"); p.print(sensorData.tvoc, 2); p.print(",");

  }

  // Environmental Data
  p.print("\"temperature\":"); p.print(sensorData.temperature, 1); p.print(",");
  p.print("\"humidity\":"); p.print(sensorData.humidity, 1); p.print(",");

  // Power Data
  p.print("\"voltage\":"); p.print(sensorData.voltage, 2); p.print(",");
  p.print("\"current_amp\":"); p.print(sensorData.current, 3); p.print(",");
  p.print("\"power_watt\":"); p.print(sensorData.power, 2); p.print(",");

  // Air Quality Index
  p.print("\"aqi\":"); p.print(sensorData.aqi); p.print(",");
  p.print("\"aqi_category\":\""); p.print(sensorData.aqi_category); p.print("\",");
  p.print("\"dominant_pollutant\":\""); p.print(sensorData.dominant_pollutant); p.print("\"");
  
  p.print("}");
}

// Send data to ESP32 via Serial2
bool sendDataToESP32() {
  if (!esp32_ready) {
    Serial.println(F("ESP32 not ready!"));
    return false;
  }

  // Show data sending icon
  display.setDrawColor(1); display.drawXBMP(75, 0, 8, 8, dataicon);
  display.sendBuffer();

  Serial.println(F("Sending data to ESP32..."));
  Serial.print(F("Payload: "));
  // Debug print to Serial
  streamJson(Serial); 
  Serial.println(); // Newline for debug view

  // Send to ESP32 with newline terminator
  streamJson(ESP32_SERIAL);
  ESP32_SERIAL.println(); // Terminate with newline

  // Wait for response (timeout 10 seconds)
  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < 10000) {
    if (ESP32_SERIAL.available()) {
      response = ESP32_SERIAL.readStringUntil('\n');
      response.trim();
      
      // Ignore intermediate status messages that might pop up
      if (response == "WIFI_OK" || response == "WIFI_FAIL" || response.startsWith("CMD:")) {
          Serial.print(F("(Ignored while waiting for ACK: "));
          Serial.print(response);
          Serial.println(F(")"));
          // Reset timer? Maybe not, strict timeout is better.
          continue; 
      }
      
      break;
    }
    delay(50);
    wdt_reset(); // Feed watchdog during wait
  }

  // Clear data icon
  // Clear data icon
  display.setDrawColor(1); display.drawXBMP(75, 0, 8, 8, blank);
  display.sendBuffer();

  if (response == "OK") {
    Serial.println(F("ESP32: Data sent successfully!"));
    return true;
  } else if (response == "FAIL") {
    Serial.println(F("ESP32: Failed to send data!"));
    return false;
  } else {
    Serial.print(F("ESP32: Unknown response: "));
    Serial.println(response);
    return false;
  }
}

void checkESP32Messages() {
  while (ESP32_SERIAL.available()) {
    String message = ESP32_SERIAL.readStringUntil('\n');
    message.trim();

    Serial.print(F("ESP32 Message: "));
    Serial.println(message);

    // Handle status messages
    if (message == "WIFI_OK") {
      wifi_status = 1;
      esp32_ready = true;
      jiofi_fail_count = 0; // Reset failure counter on success
      Serial.println(F("WiFi connected!"));
    } else if (message == "WIFI_FAIL") {
      wifi_status = 0;
      Serial.println(F("WiFi disconnected!"));
      
      // Skip restart if already in a restart process
      if (!jiofi_restarting) {
        // Auto-restart JioFi if not exceeded max retries
        jiofi_fail_count++;
        Serial.print(F("JioFi fail count: "));
        Serial.print(jiofi_fail_count);
        Serial.print(F("/"));
        Serial.println(JIOFI_MAX_RETRIES);
        
        if (jiofi_fail_count <= JIOFI_MAX_RETRIES) {
          Serial.println(F("Auto-restarting JioFi..."));
          jiofi_restarting = true;
          connectJioFi();      // Power cycle JioFi
          hardResetESP32();    // Reset ESP32 to reconnect WiFi
          jiofi_restarting = false;
        } else {
          Serial.println(F("Max JioFi retries reached. Running in offline mode."));
          display.clearBuffer();
          centerText(F("JioFi Failed"), 20);
          centerText(F("Offline Mode"), 35);
          display.sendBuffer();
          wdtDelay(3000);
          display.clearBuffer();
        }
      } else {
        Serial.println(F("(Restart already in progress, skipping)"));
      }
    } else if (message == "READY") {
      esp32_ready = true;
      Serial.println(F("ESP32 is ready!"));

      // Clear OLED display
      display.setDrawColor(0); display.drawBox(0, 11, SCREEN_WIDTH, 54); display.setDrawColor(1);;
      display.sendBuffer();
      
      // If OTA was in progress and ESP32 just rebooted, OTA is done!
      if (otaInProgress) {
        Serial.println(F("ESP32 rebooted after OTA - update successful!"));
        display.clearBuffer();
        centerText(F("OTA Complete!"), 20);
        centerText(F("ESP32 Rebooted OK"), 40);
        display.sendBuffer();
        wdtDelay(3000);
        otaInProgress = false; // Unlock display
        display.clearBuffer();
        // Force immediate full display refresh
        clear_area_update();
        oled_display_update();
        drawWiFiIcon();
        display.sendBuffer();
      }
    }
    // Handle SSE commands from server (via ESP32)
    else if (message.startsWith("CMD:")) {
      
      // Check for SD Status first
      if (message.startsWith("CMD:SD_STAT:")) {
          if (message.indexOf("FAIL") > 0) {
             sd_status = false;
          } else {
             sd_status = true;
          }
      }

      // Parse command format: CMD:COMMAND_NAME:VALUE
      int firstColon = message.indexOf(':');
      int secondColon = message.indexOf(':', firstColon + 1);
      
      // Handle LTE Status Commands
      if (message == "CMD:LTE_OK") {
        Serial.println(F("✅ LTE Status: Attached"));
        display.clearBuffer();
        centerText(F("WiFi Connected"), 0);
        centerText(F("LTE Status:"), 20);
        centerText(F("Attached"), 32);
        display.sendBuffer();
        wdtDelay(3000); // Watchdog-safe
        display.clearBuffer();
      } else if (message == "CMD:LTE_FAIL") {
        Serial.println(F("❌ LTE Status: Not Attached"));
        
        // Skip restart if already in a restart process
        if (!jiofi_restarting) {
          display.clearBuffer();
          centerText(F("LTE Not Attached"), 8);
          centerText(F("Restarting JioFi.."), 24);
          display.sendBuffer();
          wdtDelay(2000);
          
          // Auto-restart JioFi on LTE failure
          jiofi_fail_count++;
          Serial.print("JioFi/LTE fail count: ");
          Serial.print(jiofi_fail_count);
          Serial.print("/");
          Serial.println(JIOFI_MAX_RETRIES);
          
          if (jiofi_fail_count <= JIOFI_MAX_RETRIES) {
            jiofi_restarting = true;
            connectJioFi();      // Power cycle JioFi
            hardResetESP32();    // Reset ESP32 to reconnect WiFi
            jiofi_restarting = false;
          } else {
            Serial.println(F("Max JioFi/LTE retries reached. Continuing..."));
            display.clearBuffer();
            centerText(F("LTE Failed"), 20);
            centerText(F("Max Retries Hit"), 35);
            display.sendBuffer();
            wdtDelay(3000);
            display.clearBuffer();
          }
        } else {
          Serial.println("(LTE restart already in progress, skipping)");
        }
      }

      else if (firstColon > 0) {
        String command = message.substring(firstColon + 1);

        if (secondColon > 0) {
          // Command with value
          String cmdName = message.substring(firstColon + 1, secondColon);
          String cmdValue = message.substring(secondColon + 1);

          if (cmdName == "SET_INTERVAL") {
            long newInterval = cmdValue.toInt();

            Serial.println(F("═══════════════════════════════════"));
            Serial.print(F("⏰ Current Interval: "));
            Serial.print(currentSendInterval);
            Serial.print(F(" ms ("));
            Serial.print(currentSendInterval / 1000);
            Serial.println(F(" seconds)"));
            Serial.print(F("⏰ New Interval: "));
            Serial.print(newInterval);
            Serial.print(F(" ms ("));
            Serial.print(newInterval / 1000);
            Serial.println(F(" seconds)"));

            if (newInterval >= 5000) { // Minimum 5 seconds
              long oldInterval =
                  currentSendInterval; // Save old value for display

              data_send.setdelay(newInterval);
              currentSendInterval = newInterval; // Update tracking variable
              saveIntervalToEEPROM(currentSendInterval); // Save to EEPROM
              Serial.println(F("✅ SSE Command: Interval updated successfully!"));
              Serial.println(F("═══════════════════════════════════"));

              // Show feedback on OLED
              display.clearBuffer();
              centerText(F("Config Updated!"), 0);
              //display.setTextSize(...);
              //display.setTextColor(...);
              display.setCursor(10, 16);
              display.print(F("Old: "));
              display.print(oldInterval / 1000);
              display.print(F("s"));
              display.setCursor(10, 28);
              display.print(F("New: "));
              display.print(newInterval / 1000);
              display.print(F("s"));
              display.sendBuffer();
              delay(3000);
              display.clearBuffer();
            } else {
              Serial.print(F("❌ Invalid interval: "));
              Serial.print(newInterval);
              Serial.println(F(" ms (minimum 5000ms)"));
              Serial.println(F("═══════════════════════════════════"));
            }
          }
          else if (cmdName == "VERSION") {
            Serial.println(F("═══════════════════════════════════"));
            Serial.print(F("📦 ESP32 Firmware: v"));
            Serial.println(cmdValue);
            Serial.println(F("═══════════════════════════════════"));
            
            // Show on OLED briefly
            display.setDrawColor(0); display.drawBox(0, 11, SCREEN_WIDTH, 54); display.setDrawColor(1);;
            centerText(F("ESP32 Firmware"), 20);
            String verStr = "v" + cmdValue;
            centerText(verStr, 38);
            display.sendBuffer();
            wdtDelay(2000);
            display.clearBuffer();
          }
        } else {
          // Command without value
          if (command == "RESET") {
            Serial.println(F("✅ SSE Command: Reset requested"));

            // Show reset message
            display.clearBuffer();
            centerText(F("Resetting..."), 20);
            display.sendBuffer();
            delay(1000);

            // Perform soft reset (restart ESP32 connection)
            esp32_ready = false;
            wifi_status = 0;
            Serial.println(F("Device reset complete"));
          }
          else if (command == "OTA_CHECK") {
            Serial.println(F("ESP32 checking for OTA update..."));
            otaIconState = 1; // Checking
            // Show Update Check icon at x=102
            display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapUpdate_Check);
            display.sendBuffer();
          }
          else if (command == "OTA_START") {
            Serial.println(F("ESP32 OTA update started!"));
            otaInProgress = true; // Lock display
            otaIconState = 2; // Downloading
            display.clearBuffer();
            // Show Download icon at x=102
            display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapOTA_udate);
            
            centerText(F("OTA Update"), 10);
            centerText(F("Updating ESP32..."), 30);
            centerText(F("Do NOT power off!"), 50);
            display.sendBuffer();
          }
          else if (command == "OTA_DONE") {
            Serial.println(F("ESP32 OTA update complete!"));
            display.clearBuffer();
            otaIconState = 3; // Updated
            // Show Updated icon at x=102
            display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapUpdated);

            centerText(F("OTA Complete!"), 20);
            centerText(F("ESP32 Rebooting..."), 40);
            display.sendBuffer();
            wdtDelay(3000);
            otaInProgress = false; // Unlock display
            display.clearBuffer();
            // Force immediate full display refresh
            clear_area_update();
            oled_display_update();
            drawWiFiIcon();
            display.sendBuffer();
          }
          else if (command == "OTA_FAIL") {
            Serial.println(F("ESP32 OTA update failed!"));
            otaIconState = 0; // Clear icon or keep last? Clear for fail.
            display.clearBuffer();
            centerText(F("OTA Update Failed"), 25);
            centerText(F("Resuming normal..."), 45);
            display.sendBuffer();
            wdtDelay(3000);
            otaInProgress = false; // Unlock display
            display.clearBuffer();
            // Force immediate full display refresh
            clear_area_update();
            oled_display_update();
            drawWiFiIcon();
            display.sendBuffer();
          }
          else if (command == "OTA_UPTODATE") {
            Serial.println(F("Firmware is up to date."));
            otaIconState = 3; // Updated
            Serial.print(F("OTA State set to: ")); Serial.println(otaIconState);
            // Show Updated icon at x=102 (Always display)
            display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapUpdated);
            display.sendBuffer();
          }
        }
      }
    }
    // Backward compatibility with old INTERVAL format
    else if (message.startsWith("INTERVAL ")) {
      String intervalStr = message.substring(9);
      long newInterval = intervalStr.toInt();

      if (newInterval >= 5000) {
        data_send.setdelay(newInterval);
        currentSendInterval = newInterval;         // Update tracking variable
        saveIntervalToEEPROM(currentSendInterval); // Save to EEPROM
        Serial.print(F("Updated data send interval to: "));
        Serial.print(newInterval);
        Serial.println(F(" ms"));
      }
    }
  }
}

// Draw WiFi signal icon
void drawWiFiIcon() {
  if (wifi_status == 1) {
    display.setDrawColor(1); display.drawXBMP(117, 0, 11, 8, epd_bitmap_wifi);
  } else {
    display.setDrawColor(1); display.drawXBMP(117, 0, 11, 8, epd_bitmap_no_wifi);
  }
}

void clear_area_update() {
  if (otaInProgress) return; // Don't touch display during OTA
  display.setDrawColor(0); display.drawBox(0, 14, 74, 50); display.setDrawColor(1);;  // Height: 50 (y=14 to y=64)
  if (page == 1) {
    Serial.println(F("Page 1"));
    page = 2;
  } else {
    Serial.println(F("Page 2"));
    page = 1;
  }
}

void oled_display_update() {
  if (otaInProgress) return; // Don't touch display during OTA
  // Continuously read PM sensor data for real-time display
  if (readPMData()) {
    sensorData.pm1_0 = pmData.pm1_0;
    
    // Apply calibration offsets
    float p25 = pmData.pm2_5 + calib.pm25_offset;
    if(p25 < 0) p25 = 0;
    sensorData.pm2_5 = p25;
    
    float p10 = pmData.pm10 + calib.pm10_offset;
    if(p10 < 0) p10 = 0;
    sensorData.pm10 = p10;
  }

  // Battery icon - show empty battery (or connect voltage sensor to calculate
  // actual level)
  //display.drawBitmap(SCREEN_WIDTH - 128, 0, battery_level[0], 12, 8,
// 1, 0);

  // Note: To display actual voltage, connect a voltage sensor and update
  // sensorData.voltage Then uncomment below: //display.setTextSize(...);
  // //display.setTextColor(...);
  // display.setCursor(14, 0);
  // display.print(sensorData.voltage, 1);
  // display.print("V");

  // Draw SD Card Icon (Left of WiFi)
  // WiFi is at -11 (x=117). OTA is at x=102. SD should be at x=87.
  if (sd_status) {
    display.setDrawColor(1); display.drawXBMP(87, 0, 11, 8, card);
  } else {
    display.setDrawColor(1); display.drawXBMP(87, 0, 11, 8, No_card);
  }

  // Draw OTA Icon based on state
  if (otaIconState == 1) { // Checking
     display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapUpdate_Check);
  } else if (otaIconState == 2) { // Downloading
     display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapOTA_udate);
  } else if (otaIconState == 3) { // Updated
     display.setDrawColor(1); display.drawXBMP(102, 0, 11, 8, myBitmapUpdated);
  }

  // Draw separator line
  display.drawHLine(0, 10, SCREEN_WIDTH);
  // Draw vertical separator line
  display.drawVLine(74, 10, 54);

  // AQI Label and Value
  printText("AQI", 94, 16);
  // Center AQI Value (Size 3)
  display.setFont(u8g2_font_courB18_tf);
  display.setDrawColor(1);
  String aqiStr = String(sensorData.aqi);
  int valW = display.getStrWidth(aqiStr.c_str());
  // "AQI" text at 94 (width ~18) -> Center approx 103
  display.setCursor(103 - (valW / 2), 30);
  display.print(aqiStr);

  if (page == 1) {
    printText("PM2.5:", 0, 14);
    printText(sensorEnabled[8] ? String(sensorData.pm2_5, 1) : "OFF", 37, 14);

    printText("PM10 :", 0, 24);
    printText(sensorEnabled[7] ? String(sensorData.pm10, 1) : "OFF", 37, 24);

    printText("CO   :", 0, 34);
    float val = readCO();
    if(whoUnitMode) val = convertCOToWHO(val);
    printText(sensorEnabled[0] ? String(val, 2) : "OFF", 37, 34);

    printText("CO2  :", 0, 44);
    val = readCO2();
    if(whoUnitMode) val = convertCO2ToWHO(val);
    printText(sensorEnabled[1] ? String(val, 0) : "OFF", 37, 44);

    printText("O3   :", 0, 54);
    val = readO3();
    if(whoUnitMode) val = convertO3ToWHO(val);
    printText(sensorEnabled[2] ? String(val, 1) : "OFF", 37, 54);
  }

  if (page == 2) {
    printText("NH3  :", 0, 14);
    float val = readNH3();
    if(whoUnitMode) val = convertNH3ToWHO(val);
    printText(sensorEnabled[3] ? String(val, 2) : "OFF", 37, 14);

    printText("NO2  :", 0, 24);
    val = readNO2();
    if(whoUnitMode) val = convertNO2ToWHO(val);
    printText(sensorEnabled[4] ? String(val, 1) : "OFF", 37, 24);

    printText("SO2  :", 0, 34);
    val = readSO2();
    if(whoUnitMode) val = convertSO2ToWHO(val);
    printText(sensorEnabled[5] ? String(val, 1) : "OFF", 37, 34);

    printText("TVOC :", 0, 44);
    val = readTVOC();
    if(whoUnitMode) val = convertTVOCToWHO(val);
    printText(sensorEnabled[6] ? String(val, 1) : "OFF", 37, 44);

    printText("RH   :", 0, 54);
    printText(String(sensorData.humidity, 1), 37, 54);
  }
  
  // Update display once after all text is drawn (prevents artifacts from multiple refreshes)
  display.sendBuffer();
}

void esp32_status_update() {
  // Request status from ESP32
  // Request status from ESP32
  ESP32_SERIAL.println("STATUS");
  ESP32_SERIAL.println("CMD:SD_STAT"); // Also check SD status
  checkESP32Messages();
  drawWiFiIcon();
  display.sendBuffer();
}

void data_send_update() {
  // Generate random test data
  generateTestData();
  display.setDrawColor(0); display.drawBox(84, 30, 120, 42); display.setDrawColor(1);;
  display.sendBuffer();

  // Send data to ESP32
  Serial.println(F("\n========== Sending Test Data =========="));
  Serial.print(F("Current Send Interval: "));
  Serial.print(currentSendInterval);
  Serial.print(F(" ms ("));
  Serial.print(currentSendInterval / 1000);
  Serial.println(F(" seconds)"));

  bool success = sendDataToESP32();

  if (success) {
    Serial.println(F("========== Data sent successfully! ==========\n"));
    esp32_fail_count = 0; // Reset watchdog counter
  } else {
    Serial.println(F("========== Failed to send data! ==========\n"));
    esp32_fail_count++;
    Serial.print(F("Failure count: "));
    Serial.print(esp32_fail_count);
    Serial.println(F("/2"));

    if (esp32_fail_count >= 2) {
      Serial.println(F("Watchdog: Too many failures! Triggering System Reset..."));
      esp32_fail_count = 0;
      hardResetESP32();
    }
  }
}

// ===== MENU & CALIBRATION SYSTEM =====

void checkEncoder();
void drawMenu();

// Menu Variables
int menuState = 0; // 0=None, 1=Main, 2=CalibSelect, 3=SetPPM, 4=Sampling, 5=NetInfo, 6=Reset
int menuSelection = 0;
int maxSelection = 0;
bool redraw = true;
int selectedSensorIndex = 0;
float targetPPM = 0.0;
bool displayInPPB = false; // Track current display unit
unsigned long calibStartTime = 0;

int calibMode = 0; // 0=Zero, 1=Span, 2=R0
const bool nativeIsPPB[] = {false, false, true, false, true, true, true, false, false, false}; // true=PPB, false=PPM




// Display Helper
void centerText(String text, int y, int size = 1) {
  if (size == 1) display.setFont(u8g2_font_6x10_tf);
  else if (size == 2) display.setFont(u8g2_font_7x14B_tf);
  else display.setFont(u8g2_font_courB18_tf);

  int w = display.getStrWidth(text.c_str());
  int x = (128 - w) / 2;
  
  display.setDrawColor(1);
  display.setCursor(x, y);
  display.print(text);
}

// Perform Calculation and Save
void performCalibration() {
  display.clearBuffer();
  centerText(F("Calibrating..."), 20, 1);
  display.sendBuffer();

  // Disable ABC during calibration to avoid interference
  abcEnabled = false;
  long rawSum = 0;
  int samples = 20;
  int pin = 0;
  float rl = 10.0;

  // Map index to PIN and RL
  switch(selectedSensorIndex) {
    case 0: pin = MQ7_PIN; rl = RL_MQ7; break;
    case 1: pin = MQ135_PIN; rl = RL_MQ135; break;
    case 2: pin = MQ131_PIN; rl = RL_MQ131; break;
    case 3: pin = MQ137_PIN; rl = RL_MQ137; break;
    case 4: pin = NO2_PIN; rl = 0; break;
    case 5: pin = MQ2_PIN; rl = RL_MQ2; break;
    case 6: pin = VOC_PIN; rl = 0; break;
  }

  float voltage = 0;

  if (selectedSensorIndex == 0) {
    // === MQ-7 SPECIAL: Phase-aware calibration with PWM filtering ===
    
    // If in heating phase, wait for sensing phase
    if (mq7Phase == MQ7_HEATING) {
      unsigned long remaining = MQ7_HEAT_DURATION - (millis() - mq7PhaseStart);
      display.clearBuffer();
      centerText(F("MQ7: Waiting for"), 10);
      centerText(F("sensing phase..."), 22);
      display.sendBuffer();
      
      // Wait with countdown
      while (mq7Phase == MQ7_HEATING) {
        wdt_reset();
        updateMQ7Heater();  // Keep cycling running
        unsigned long left = 0;
        if (millis() - mq7PhaseStart < MQ7_HEAT_DURATION)
          left = (MQ7_HEAT_DURATION - (millis() - mq7PhaseStart)) / 1000;
        display.setDrawColor(0); display.drawBox(0, 36, SCREEN_WIDTH, 16); display.setDrawColor(1);;
        centerText(String(left) + "s remaining", 40);
        display.sendBuffer();
        delay(500);
      }
    }
    
    // Now in sensing phase — wait 30s for stabilization
    display.clearBuffer();
    centerText(F("MQ7: Stabilizing"), 10);
    centerText(F("30s wait..."), 22);
    display.sendBuffer();
    
    for (int i = 30; i > 0; i--) {
      wdt_reset();
      display.setDrawColor(0); display.drawBox(0, 36, SCREEN_WIDTH, 16); display.setDrawColor(1);;
      centerText(String(i) + "s remaining", 40);
      display.sendBuffer();
      delay(1000);
    }
    
    // Take PWM-filtered samples (same logic as readCO)
    display.clearBuffer();
    centerText(F("MQ7: Sampling..."), 20);
    display.sendBuffer();
    
    int validCount = 0;
    float voltageSum = 0.0;
    int totalSamples = 100;  // More samples for calibration accuracy
    for (int i = 0; i < totalSamples; i++) {
      wdt_reset();
      int adc = analogRead(MQ7_PIN);
      float v = adc * (5.0 / 1023.0);
      if (v < 3.0) {  // Filter PWM-off artifacts
        voltageSum += v;
        validCount++;
      }
      delayMicroseconds(200);
    }
    
    if (validCount > 0) {
      voltage = voltageSum / validCount;
    } else {
      voltage = 0.01;
    }
    
    Serial.print(F("MQ7 CALIB: valid="));
    Serial.print(validCount);
    Serial.print(F("/"));
    Serial.print(totalSamples);
    Serial.print(F(" avgV="));
    Serial.println(voltage, 3);
    
  } else {
    // === Standard sampling for all other sensors ===
    for(int i=0; i<samples; i++) {
      rawSum += analogRead(pin);
      delay(100);
    }
    float avgRaw = rawSum / (float)samples;
    voltage = avgRaw * (5.0 / 1023.0);
  }
  
  if(voltage < 0.01) voltage = 0.01;
  
  // Calculate Calibration based on Mode
  if (calibMode == 0) {
      // === Zero Calibration ===
      // Target is 0. We want the current raw reading to map to 0.
      // Current formula: val = (raw - zero) * span
      // Reverse: raw = (val_from_read / span) + zero
      // To make new_val = 0 -> (raw - new_zero) * span = 0 => new_zero = raw
      
      // We need to call the readX() function to get the current "val" that includes the OLD calibration
      // Then reverse it to get "raw".
      
      float currentRead = 0;
      switch(selectedSensorIndex) {
        case 0: currentRead = readCO(); break;
        case 1: currentRead = readCO2(); break;
        case 2: currentRead = readO3(); break;
        case 3: currentRead = readNH3(); break;
        case 4: currentRead = readNO2(); break;
        case 5: currentRead = readSO2(); break;
        case 6: currentRead = readTVOC(); break;
      }
      
      float currentSpan = calib.span[selectedSensorIndex];
      float currentZero = calib.zero[selectedSensorIndex];
      
      // Reverse calc to get 'raw' (after T/H comp etc, but before 2-point)
      float raw = (currentRead / currentSpan) + currentZero;
      
      calib.zero[selectedSensorIndex] = raw;
      // Optionally reset span? Standard industry practice favors keeping span or resetting to 1.
      // Let's reset span to 1.0 to avoid compounding errors if Zero was way off.
      // But if user wants to just zero-shift, they might want to keep span.
      // Decision: Reset Span is safer for "Zero Calibration".
      calib.span[selectedSensorIndex] = 1.0;
      
      Serial.print(F("CALIB ZERO: Sensor ")); Serial.print(selectedSensorIndex);
      Serial.print(F(" New Zero=")); Serial.println(raw);
  }
  else if (calibMode == 1) {
      // === Span Calibration ===
      // Target is targetPPM.
      // We want: (raw - zero) * new_span = target
      // raw - zero = currentRead / currentSpan
      // new_span = target / (currentRead / currentSpan) = (target * currentSpan) / currentRead
      
      float currentRead = 0;
      switch(selectedSensorIndex) {
        case 0: currentRead = readCO(); break;
        case 1: currentRead = readCO2(); break;
        case 2: currentRead = readO3(); break;
        case 3: currentRead = readNH3(); break;
        case 4: currentRead = readNO2(); break;
        case 5: currentRead = readSO2(); break;
        case 6: currentRead = readTVOC(); break;
      }
      
      if (currentRead < 0.001) currentRead = 0.001; // Avoid divide by zero
      
      float oldSpan = calib.span[selectedSensorIndex];
      float newSpan = (targetPPM * oldSpan) / currentRead;
      
      calib.span[selectedSensorIndex] = newSpan;
      
      Serial.print(F("CALIB SPAN: Sensor ")); Serial.print(selectedSensorIndex);
      Serial.print(F(" New Span=")); Serial.println(newSpan);
  }
  else {
      // === R0 Calibration (Original Logic) ===
      
      // Calculate new R0/Baseline
      float newVal = 0;
      
      // Convert targetPPM to Native Unit for calculation
      float calcTarget = targetPPM;
      bool native = nativeIsPPB[selectedSensorIndex];
      
      if (displayInPPB && !native) { 
         // Displayed in PPB, Native is PPM -> Convert to PPM
         calcTarget = targetPPM / 1000.0;
      } else if (!displayInPPB && native) {
         // Displayed in PPM, Native is PPB -> Convert to PPB
         calcTarget = targetPPM * 1000.0;
      }
    
      if (selectedSensorIndex == 0) { // CO (MQ7)
        float rs = ((5.0 * rl) / voltage) - rl;
        float K = getCorrectionFactor(0, sensorData.temperature, sensorData.humidity);
        float Rs_comp = rs / K; 
        
        float exponent = -0.36 * log10(calcTarget) + 0.74;
        float expectedRatio = pow(10, exponent);
        newVal = Rs_comp / expectedRatio;
        calib.r0_co = newVal;
        Serial.print(F("MQ7 CALIB: Rs="));
        Serial.print(rs, 3);
        Serial.print(F(" R0="));
        Serial.println(newVal, 3);
      }
      else if (selectedSensorIndex == 1) { // CO2 (MQ135)
        float rs = ((5.0 * rl) / voltage) - rl;
        float K = getCorrectionFactor(1, sensorData.temperature, sensorData.humidity);
        float Rs_comp = rs / K; 
        
        float expectedRatio = pow((calcTarget / 116.602), (1.0 / -2.769));
        newVal = Rs_comp / expectedRatio;
        calib.r0_co2 = newVal;
      }
      else if (selectedSensorIndex == 2) { // O3 (MQ131)
         float rs = ((5.0 * rl) / voltage) - rl;
        float K = getCorrectionFactor(2, sensorData.temperature, sensorData.humidity);
        float Rs_comp = rs / K; 
         
         float exponent = -0.44 * log10(calcTarget) - 0.92;
         float expectedRatio = pow(10, exponent);
         newVal = Rs_comp / expectedRatio;
         calib.r0_o3 = newVal;
      }
      else if (selectedSensorIndex == 3) { // NH3 (MQ137)
        float rs = ((5.0 * rl) / voltage) - rl;
        float K = getCorrectionFactor(3, sensorData.temperature, sensorData.humidity);
        float Rs_comp = rs / K; 
        
        float expectedRatio = pow((calcTarget/50.0), (1.0/-1.2));
        newVal = Rs_comp / expectedRatio;
        calib.r0_nh3 = newVal;
      }
      else if (selectedSensorIndex == 4) { // NO2 (Fermion)
        newVal = voltage; // Baseline Voltage
        calib.r0_no2 = newVal;
      }
      else if (selectedSensorIndex == 5) { // SO2 (MQ2)
        float rs = ((5.0 * rl) / voltage) - rl;
        float K = getCorrectionFactor(5, sensorData.temperature, sensorData.humidity);
        float Rs_comp = rs / K; 
        
        float expectedRatio = pow((calcTarget/50.0), (1.0/-1.4));
        newVal = Rs_comp / expectedRatio;
        calib.r0_so2 = newVal;
      }
      else if (selectedSensorIndex == 6) { // TVOC
        newVal = voltage; // Baseline
        calib.r0_tvoc = newVal;
      }
      else if (selectedSensorIndex == 7) { // PM10
          readPMData(); // Refresh raw readings
          float raw = pmData.pm10;
          calib.pm10_offset = calcTarget - raw;
      }
      else if (selectedSensorIndex == 8) { // PM2.5
          readPMData(); // Refresh raw readings
          float raw = pmData.pm2_5;
          calib.pm25_offset = calcTarget - raw;
      }
      else if (selectedSensorIndex == 9) { // PM1.0
          readPMData(); // Refresh raw readings
          float raw = pmData.pm1_0;
          calib.pm1_offset = calcTarget - raw;
      }
  }

  saveCalib();
  delay(1000);

  // Re-enable ABC and reset window after calibration
  abcEnabled = true;
  abcMinNO2 = 99.0;
  abcMinTVOC = 99.0;
  abcWindowStart = millis();

  menuState = 2; // Back to sensor select
  redraw = true;
}

// Reset calibration to defaults
void resetCalibration() {
    display.clearBuffer();
    centerText(F("Resetting..."), 20, 1);
    display.sendBuffer();
    
    // Restore defaults
    calib = { 
        10.0, 76.63, 100.0, 50.0, 9.83, 2.5, 0.6, 
        0.0, 0.0, 0.0, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {1,1,1,1,1,1,1,1,1,1}, 
        0};
        
    // Force write to EEPROM
    saveCalib();
    
    Serial.println(F(">>> Calibration reset to defaults."));
    delay(2000);
    
    menuState = 6;
    redraw = true;
}


void handleMenu() {
  if (!menuActive) return;
  wdt_reset(); // Feed watchdog while menu is active (loop() is blocked)

  if (redraw) {
    display.clearBuffer();
    //display.setTextSize(...);
    //display.setTextColor(...);

    int lineHeight = 10; // 8px font + 5px space
    int startY = 12; // Title height + space

    if (menuState == 1) { // Main Menu
      centerText(F("=== MENU ==="), 0, 1);
      display.drawLine(0, 9, 128, 9);
      
      const char* mainItems[] = {"1. Calibrate", "2. Sensors", "3. Net Config", "4. Unit System", "5. SD Card", "6. Reset", "7. About", "8. Exit"};
      int totalItems = 8;
      maxSelection = totalItems - 1;

      // Scrolling Window (Show 5 lines max)
      int windowSize = 5;
      int startIdx = 0;
      if (menuSelection >= windowSize) startIdx = menuSelection - windowSize + 1;

      for(int i=0; i<windowSize; i++) {
         int idx = startIdx + i;
         if (idx < totalItems) {
             int y = startY + (i * lineHeight);
             display.setCursor(0, y);
             if(idx == menuSelection) display.print(F("> ")); else display.print(F("  "));
             display.print(mainItems[idx]);
         }
      }
    }
    else if (menuState == 10) { // SD Card Menu
       centerText(F("= SD CARD ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
       const char* sdItems[] = {"1. Status", "2. Clear Logs", "3. Clear Data", "4. Format SD", "5. Back"};
       int totalItems = 5;
       maxSelection = totalItems - 1;
       
       for(int i=0; i<totalItems; i++) {
           int y = startY + (i * lineHeight);
           display.setCursor(0, y);
           if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
           display.print(sdItems[i]);
       }
    }
    else if (menuState == 2) { // Calibration Select
      centerText(F("= SENSORS ="), 0, 1);
      display.drawLine(0, 9, 128, 9);

      int totalItems = 11; // 10 Sensors + Back
      maxSelection = totalItems - 1;
      
      int windowSize = 5;
      int startIdx = 0;
      if (menuSelection >= windowSize) startIdx = menuSelection - windowSize + 1;

      for(int i=0; i<windowSize; i++) {
         int idx = startIdx + i;
         if (idx < totalItems) {
            int y = startY + (i * lineHeight);
            display.setCursor(0, y);
            if(idx == menuSelection) display.print(F("> ")); else display.print(F("  "));
            
            if(idx < 10) display.print(sensorNames[idx]);
            else display.print(F("Back"));
         }
      }
    }

    else if (menuState == 21) { // Calibration Mode Select
       centerText(F("= CALIB MODE ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
       const char* optItems[] = {"1. Zero Cal (0 ppm)", "2. Span Cal (Std)", "3. R0 / Baseline", "4. Back"};
       int totalItems = 4;
       maxSelection = totalItems - 1;
       
       for(int i=0; i<totalItems; i++) {
           int y = startY + (i * lineHeight);
           display.setCursor(0, y);
           if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
           display.print(optItems[i]);
       }
    }

    else if (menuState == 3) { // Calibration Set PPM

      centerText(F("= SET TARGET ="), 0, 1);
      display.drawLine(0, 9, 128, 9);
      
      display.setCursor(0, 15);
      display.print(F("Sensor: ")); display.println(sensorNames[selectedSensorIndex]);

      // Calculate Current Value
      float currentVal = 0.0;
      switch(selectedSensorIndex) {
        case 0: currentVal = readCO(); break;
        case 1: currentVal = readCO2(); break;
        case 2: currentVal = readO3(); break;
        case 3: currentVal = readNH3(); break;
        case 4: currentVal = readNO2(); break;
        case 5: currentVal = readSO2(); break;
        case 6: currentVal = readTVOC(); break;
        case 7: currentVal = sensorData.pm10; break;
        case 8: currentVal = sensorData.pm2_5; break;
        case 9: currentVal = sensorData.pm1_0; break;
      }

      // Display Current

      display.setCursor(0, 25);
      display.print(F("Curr: ")); 
      
      // Convert Current Reading for Display
      bool native = nativeIsPPB[selectedSensorIndex];
      float displayVal = currentVal;
      String unitStr = "";
      
      if (selectedSensorIndex >= 7) {
        displayVal = currentVal; // PM Sensors always ug/m3
        unitStr = F(" ug/m3");
      } else if (whoUnitMode) {
         // WHO Mode
         switch(selectedSensorIndex) {
            case 0: displayVal = convertCOToWHO(currentVal); unitStr = F(" mg/m3"); break;
            case 1: displayVal = convertCO2ToWHO(currentVal); unitStr = F(" mg/m3"); break;
            case 2: displayVal = convertO3ToWHO(currentVal); unitStr = F(" ug/m3"); break;
            case 3: displayVal = convertNH3ToWHO(currentVal); unitStr = F(" mg/m3"); break;
            case 4: displayVal = convertNO2ToWHO(currentVal); unitStr = F(" ug/m3"); break;
            case 5: displayVal = convertSO2ToWHO(currentVal); unitStr = F(" ug/m3"); break;
            case 6: displayVal = convertTVOCToWHO(currentVal); unitStr = F(" ug/m3"); break;
         }
      } else {
        // EPA Mode
        if (displayInPPB && !native) displayVal *= 1000.0;
        else if (!displayInPPB && native) displayVal /= 1000.0;
        unitStr = (displayInPPB ? F(" ppb") : F(" ppm"));
      }
      
      display.print(displayVal, 1);
      display.println(unitStr);

      
      
      // Display Target
      display.setCursor(0, 35);
      display.print(F("Tgt:  ")); 
      
      float tgtDisplay = targetPPM;
      String tgtUnitStr = "";
      
      if (selectedSensorIndex >= 7) {
         tgtDisplay = targetPPM;
         tgtUnitStr = F(" ug/m3");
      } else if (whoUnitMode) {
         // WHO Mode Target Conversion
         switch(selectedSensorIndex) {
            case 0: tgtDisplay = convertCOToWHO(targetPPM); tgtUnitStr = F(" mg/m3"); break;
            case 1: tgtDisplay = convertCO2ToWHO(targetPPM); tgtUnitStr = F(" mg/m3"); break;
            case 2: tgtDisplay = convertO3ToWHO(targetPPM); tgtUnitStr = F(" ug/m3"); break;
            case 3: tgtDisplay = convertNH3ToWHO(targetPPM); tgtUnitStr = F(" mg/m3"); break;
            case 4: tgtDisplay = convertNO2ToWHO(targetPPM); tgtUnitStr = F(" ug/m3"); break;
            case 5: tgtDisplay = convertSO2ToWHO(targetPPM); tgtUnitStr = F(" ug/m3"); break;
            case 6: tgtDisplay = convertTVOCToWHO(targetPPM); tgtUnitStr = F(" ug/m3"); break;
         }
      } else {
         // EPA Mode Target
         if (displayInPPB && !native) tgtDisplay *= 1000.0;
         else if (!displayInPPB && native) tgtDisplay /= 1000.0;
         tgtUnitStr = (displayInPPB ? F(" ppb") : F(" ppm"));
      }
      
      display.print(tgtDisplay, 1);
      display.println(tgtUnitStr);

      
      centerText(F("[Click for Options]"), 55, 1);
      maxSelection = 10000;
    }
    else if (menuState == 31) { // Confirm Calibration Action
       centerText(F("= ACTION ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
       const char* actItems[] = {"1. Save & Exit", "2. Continue Edit", "3. Exit (No Save)"};
       int totalItems = 3;
       maxSelection = totalItems - 1;
       
       for(int i=0; i<totalItems; i++) {
         int y = startY + (i * lineHeight);
         display.setCursor(0, y);
         if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
         display.print(actItems[i]);
       }
    }




    else if (menuState == 5) { // Network Config Submenu
      centerText(F("= NET CONFIG ="), 0, 1);
      display.drawLine(0, 9, 128, 9);

      const char* netItems[] = {"1. Status", "2. SSID", "3. Password", "4. Back"};
      int totalItems = 4;
      maxSelection = totalItems - 1;
      
      for(int i=0; i<totalItems; i++) {
        int y = startY + (i * lineHeight);
        display.setCursor(0, y);
        if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
        display.print(netItems[i]);
      }
    }
    else if (menuState == 51) { // Net: Status
       centerText(F("= NET STATUS ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       centerText(wifi_status ? F("Connected") : F("Disconnected"), 25, 1);
       centerText(F("[Click to Back]"), 50, 1);
    }
    else if (menuState == 52) { // Net: SSID
       centerText(F("= SSID ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       centerText(F("SSID: JIOFI_xxxx"), 20, 1);
       centerText(F("(View Only)"), 35, 1);
       centerText(F("[Click to Back]"), 50, 1);
    }
    else if (menuState == 53) { // Net: Password
       centerText(F("= PASSWORD ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       centerText(F("Pass: ********"), 20, 1);
       centerText(F("(View Only)"), 35, 1);
       centerText(F("[Click to Back]"), 50, 1);
    }
    else if (menuState == 6) { // Reset
       centerText(F("= RESET ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
        const char* rstItems[] = {"1. System Reboot", "2. Reset Params", "3. Back"};
        int totalItems = 3;
        maxSelection = totalItems - 1;
        
        for(int i=0; i<totalItems; i++) {
           int y = 20 + (i * lineHeight);
           display.setCursor(0, y);
           if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
           display.print(rstItems[i]);
        }
    }
    else if (menuState == 7) { // About
       centerText(F("= ABOUT ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       centerText(F("AQI Monitor v1.2"), 20, 1);
       centerText(F("Dev: Mega + ESP32"), 32, 1);
       centerText(F("[Click to Back]"), 50, 1);
    }
    else if (menuState == 9) { // Unit System
       centerText(F("= UNIT SYSTEM ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
       // Create a list menu structure
       const char* unitItems[] = {
         whoUnitMode ? "Mode: WHO (ug/m3)" : "Mode: EPA (ppm)", 
         "Back"
       };
       int totalItems = 2;
       maxSelection = totalItems - 1;
       
       for(int i=0; i<totalItems; i++) {
          int y = 20 + (i * lineHeight); // Start a bit lower
          display.setCursor(0, y);
          if(i == menuSelection) display.print(F("> ")); else display.print(F("  "));
          display.print(unitItems[i]);
       }
    }
    else if (menuState == 8) { // Sensor Enable/Disable
       centerText(F("= SENSORS ="), 0, 1);
       display.drawLine(0, 9, 128, 9);
       
       const char* sensorLabels[] = {"CO", "CO2", "O3", "NH3", "NO2", "SO2", "TVOC", "PM10", "PM2.5", "PM1.0", "Back"};
       int totalItems = 11; // 10 sensors + Back
       maxSelection = totalItems - 1;
       
       // Scrolling window (5 visible items)
       int windowSize = 5;
       int startIdx = 0;
       if (menuSelection >= windowSize) startIdx = menuSelection - windowSize + 1;
       
       for(int i = 0; i < windowSize; i++) {
         int idx = startIdx + i;
         if (idx < totalItems) {
           int y = startY + (i * lineHeight);
           display.setCursor(0, y);
           if (idx == menuSelection) display.print(F("> ")); else display.print(F("  "));
           
           if (idx < NUM_SENSORS) {
             // Show sensor name with ON/OFF status
             display.print(sensorLabels[idx]);
             // Right-align the status
             int labelLen = strlen(sensorLabels[idx]);
             int padding = 10 - labelLen; // Pad to column 10
             for (int p = 0; p < padding; p++) display.print(F(" "));
             display.print(sensorEnabled[idx] ? F("[ON]") : F("[OFF]"));
           } else {
             display.print(sensorLabels[idx]); // "Back"
           }
         }
       }
    }

    display.sendBuffer();
    redraw = false;
  }
  
  checkEncoder();
}


void checkEncoder() {
  int clkState = digitalRead(ENC_CLK);
  
  if (clkState != lastClk && clkState == LOW) { // Falling edge
    float inc = 1.0;
    if (menuState == 3) {
       // Per-sensor step sizes for precise calibration
       switch(selectedSensorIndex) {
         case 0:  // CO (ppm) - 0.1 below 10, 1.0 above
           inc = (targetPPM < 10) ? 0.1 : 1.0;
           break;
         case 1:  // CO2 (ppm) - always integer steps
           inc = (targetPPM < 100) ? 1.0 : 10.0;
           break;
         case 2:  // O3 (ppb native) 
           if (displayInPPB) inc = (targetPPM < 100) ? 1.0 : 10.0;
           else inc = 0.01;  // ppm display
           break;
         case 3:  // NH3 (ppm) - 0.1 below 10, 1.0 above
           inc = (targetPPM < 10) ? 0.1 : 1.0;
           break;
         case 4:  // NO2 (ppb native)
           if (displayInPPB) inc = (targetPPM < 100) ? 1.0 : 10.0;
           else inc = 0.01;
           break;
         case 5:  // SO2 (ppb native)
           if (displayInPPB) inc = (targetPPM < 100) ? 1.0 : 10.0;
           else inc = 0.01;
           break;
         case 6:  // TVOC (ppb native)
           if (displayInPPB) inc = (targetPPM < 100) ? 1.0 : 10.0;
           else inc = 0.01;
           break;
         case 7:  // PM10 (ug/m3 offset) - 0.1 steps
         case 8:  // PM2.5 (ug/m3 offset)
         case 9:  // PM1.0 (ug/m3 offset)
           inc = (abs(targetPPM) < 10) ? 0.1 : 1.0;
           break;
       }
    }
    
    if (digitalRead(ENC_DT) != clkState) { // CW
      if (menuState == 3) targetPPM += inc;
      else if (menuSelection < maxSelection) menuSelection++;
    } else { // CCW
      if (menuState == 3) targetPPM -= inc;
      else if (menuSelection > 0) menuSelection--;
    }
    // Allow negative only for PM sensors (offset calibration)
    if (selectedSensorIndex < 7 && targetPPM < 0) targetPPM = 0;
    redraw = true;
  }
  lastClk = clkState;

  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastButtonPress > 300) { // Debounce
      lastButtonPress = millis();
      
      if (menuState == 0) { // Open Menu
        menuActive = true;
        menuState = 1;
        menuSelection = 0;
      }
      else if (menuState == 1) { // Main Menu
        if (menuSelection == 0) { menuState = 2; menuSelection = 0; } // Calib (Note: previously was 21, check logic)
        else if (menuSelection == 1) { menuState = 8; menuSelection = 0; } // Sensors
        else if (menuSelection == 2) { menuState = 5; menuSelection = 0; } // Net Config
        else if (menuSelection == 3) { menuState = 9; menuSelection = 0; } // Unit System
        else if (menuSelection == 4) { menuState = 10; menuSelection = 0; } // SD Card
        else if (menuSelection == 5) { menuState = 6; menuSelection = 0; } // Reset
        else if (menuSelection == 6) { menuState = 7; } // About
        else if (menuSelection == 7) { 
           menuActive = false; 
           menuState = 0; 
           display.clearBuffer(); 
           esp32_fail_count = 0;
        } // Exit
      }
      else if (menuState == 10) { // SD Card Menu
          if (menuSelection == 0) { // Status
             checkSDStatus();
             // Stay in menu
          }
          else if (menuSelection == 1) { // Clear Logs
             sendSDCommand("CMD:SD_DEL:log.txt", "Logs Cleared");
          }
          else if (menuSelection == 2) { // Clear Data
             sendSDCommand("CMD:SD_DEL:data.csv", "Data Cleared");
          }
          else if (menuSelection == 3) { // Format
             display.clearBuffer();
             centerText(F("Formatting..."), 20, 1);
             centerText(F("PLEASE WAIT"), 35, 1);
             display.sendBuffer();
             sendSDCommand("CMD:SD_FMT", "Format Done");
          }
          else if (menuSelection == 4) { // Back
             menuState = 1; menuSelection = 4;
          }
      }
      else if (menuState == 2) { // Sensor Select
        if (menuSelection == 10) { menuState = 1; menuSelection = 0; } // Back
        else {
           selectedSensorIndex = menuSelection;
           menuState = 21; // Go to Options
           menuSelection = 0;
           
           // Initialize Default Unit for this sensor
           displayInPPB = nativeIsPPB[selectedSensorIndex];
        }
      }

      else if (menuState == 21) { // Calibration Mode Select
         if (menuSelection == 0) { // 1. Zero Cal
             calibMode = 0;
             targetPPM = 0.0;
             // Go directly to confirmation
             menuState = 31; 
             menuSelection = 0;
         } 
         else if (menuSelection == 1) { // 2. Span Cal
             calibMode = 1;
             targetPPM = 0.0; // Reset, user sets it
             menuState = 3;   // Go to Set Target
         }
         else if (menuSelection == 2) { // 3. R0 / Baseline
             calibMode = 2;
             // Set default likely target
             if(selectedSensorIndex == 1) targetPPM = 400.0; // CO2
             else if (selectedSensorIndex == 2) targetPPM = 20.0; // O3
             else targetPPM = 0.0;
             
             // Match Display Unit
             if (displayInPPB && !nativeIsPPB[selectedSensorIndex]) targetPPM *= 1000; 
             else if (!displayInPPB && nativeIsPPB[selectedSensorIndex]) targetPPM /= 1000;
             
             menuState = 3;
         }
         else { // Back
             menuState = 2; menuSelection = 0;
         }
      }

      else if (menuState == 3) { // Go to Confirm Menu
          menuState = 31; 
          menuSelection = 0; 
          redraw = true;
      }
      else if (menuState == 31) { // Handle Confirmation
          if (menuSelection == 0) { // Save & Exit
              performCalibration(); 
          }
          else if (menuSelection == 1) { // Continue Edit
              menuState = 3; 
          }
          else if (menuSelection == 2) { // Exit No Save
              menuState = 21; // Back to Options
              menuSelection = 0;
          }
      }


      else if (menuState == 5) { // Net Config
         if (menuSelection == 0) menuState = 51; // Status
         else if (menuSelection == 1) menuState = 52; // SSID
         else if (menuSelection == 2) menuState = 53; // Pass
         else if (menuSelection == 3) { menuState = 1; menuSelection = 2; } // Back to main (Net Config index)
      }
      else if (menuState == 51 || menuState == 52 || menuState == 53) {
         menuState = 5; // Back to Net Menu
      }
      else if (menuState == 6) { // Reset Submenu
         if (menuSelection == 0) { // System Reboot
             display.clearBuffer();
             centerText(F("Rebooting..."), 20, 1);
             display.sendBuffer();
             delay(1000);
             wdt_enable(WDTO_15MS);
             while(1);
         }
         else if (menuSelection == 1) { // Reset Params
             resetCalibration();
         }
         else if (menuSelection == 2) { // Back
             menuState = 1; 
             menuSelection = 4; 
         } 
      }

      else if (menuState == 7) { // About Back
         menuState = 1; menuSelection = 5; // Back to main (About index was 4, now 5)
      }
      else if (menuState == 9) { // Unit System List
         if (menuSelection == 0) {
            // Toggle Mode
            whoUnitMode = !whoUnitMode;
            saveUnitMode();
         } 
         else if (menuSelection == 1) {
            // Back
            menuState = 1; menuSelection = 3;
         }
         redraw = true;
      }
      else if (menuState == 8) { // Sensor Enable/Disable
         if (menuSelection < NUM_SENSORS) {
           // Toggle sensor
           sensorEnabled[menuSelection] = !sensorEnabled[menuSelection];
           saveSensorConfig(); // Save to EEPROM
           Serial.print(F("Sensor "));
           Serial.print(menuSelection);
           Serial.println(sensorEnabled[menuSelection] ? F(" ENABLED") : F(" DISABLED"));
         } else {
           // Back
           menuState = 1; menuSelection = 1;
         }
      }
      redraw = true;
    }
  }
}


void connectJioFi() {
  // Turning on
  display.clearBuffer();
  centerText(F("Turning on JioFi.."), 20);
  display.sendBuffer();
  Serial.println(F("Turning on JioFi..."));
  
  pinMode(jiofi_switch, OUTPUT);
  digitalWrite(jiofi_switch, LOW);
  wdtDelay(5000);  // Watchdog-safe
  digitalWrite(jiofi_switch, HIGH);
  
  // Waiting for network
  display.clearBuffer();
  centerText(F("Waiting for Network"), 20);
  display.sendBuffer();
  
  // Wait loop handled by main loop checks
}

// ===== SD CARD HELPER FUNCTIONS =====

void checkSDStatus() {
  display.clearBuffer();
  centerText(F("Checking SD..."), 20, 1);
  display.sendBuffer();
  
  // Clear buffer
  while(ESP32_SERIAL.available()) ESP32_SERIAL.read();
  
  ESP32_SERIAL.println("CMD:SD_STAT");
  
  unsigned long start = millis();
  String resp = "";
  bool received = false;
  
  while(millis() - start < 3000) {
    if(ESP32_SERIAL.available()) {
      resp = ESP32_SERIAL.readStringUntil('\n');
      resp.trim();
      if(resp.startsWith("CMD:SD_STAT:")) {
        received = true;
        break;
      }
    }
  }
  
  display.clearBuffer();
  centerText(F("= SD STATUS ="), 0, 1);
  display.drawLine(0, 9, 128, 9);
  
  if (received) {
    if (resp.indexOf("FAIL") > 0) {
      centerText(F("SD Not Mounted"), 25, 1);
    } else {
      // Parse: CMD:SD_STAT:<rows>,<freeMB>
      int firstSep = resp.indexOf(':'); // After CMD
      int secondSep = resp.indexOf(':', firstSep + 1); // After SD_STAT
      int commaSep = resp.indexOf(',');
      
      String rows = resp.substring(secondSep + 1, commaSep);
      String freeMB = resp.substring(commaSep + 1);
      
      display.setCursor(0, 20);
      display.print(F("CSV Rows: ")); display.println(rows);
      
      display.setCursor(0, 35);
      display.print(F("Free Spc: ")); display.print(freeMB); display.println(F(" MB"));
    }
  } else {
    centerText(F("Timeout / Error"), 25, 1);
  }
  
  centerText(F("[Click to Back]"), 55, 1);
  display.sendBuffer();
  
  // Wait for button release (if still held from entering menu)
  while(digitalRead(ENC_SW) == LOW) {
    wdt_reset();
    delay(10);
  }
  delay(100); // Debounce release

  // Wait for new click
  while(digitalRead(ENC_SW) == HIGH) {
    wdt_reset();
    delay(10);
  }
  delay(300); // Debounce press
  redraw = true;
}

void sendSDCommand(String cmd, String successMsg) {
  display.clearBuffer();
  centerText(F("Sending Cmd..."), 20, 1);
  display.sendBuffer();
  
  // Clear buffer
  while(ESP32_SERIAL.available()) ESP32_SERIAL.read();
  
  ESP32_SERIAL.println(cmd);
  
  unsigned long start = millis();
  bool success = false;
  
  while(millis() - start < 5000) { // 5s timeout for formatting
    if(ESP32_SERIAL.available()) {
      String resp = ESP32_SERIAL.readStringUntil('\n');
      resp.trim();
      // Check for expected response command echo + OK
      // e.g. CMD:SD_DEL:OK
      // simple check: ends with OK
      if(resp.endsWith(":OK")) {
        success = true;
        break;
      }
      if(resp.endsWith(":FAIL")) {
        break;
      }
    }
    wdt_reset();
  }
  
  display.clearBuffer();
  if(success) {
    centerText(successMsg, 20, 1);
    centerText(F("Success!"), 35, 1);
  } else {
    centerText(F("Command Failed"), 25, 1);
  }
  display.sendBuffer();
  delay(2000);
  redraw = true;
}


// ===== EEPROM CALIBRATION FUNCTIONS =====
void saveCalib() {
  calib.initialized = 0xA6; // Mark as valid (v2)
  EEPROM.put(EEPROM_CALIB_ADDR, calib);
  Serial.println(F(">>> EEPROM: Calibration Saved."));
  Serial.print(F("    R0_CO2: ")); Serial.println(calib.r0_co2);
  Serial.print(F("    PM25_Off: ")); Serial.println(calib.pm25_offset);
}

void loadCalib() {
  EEPROM.get(EEPROM_CALIB_ADDR, calib);
  if (calib.initialized == 0xA6) {
    Serial.println(F(">>> EEPROM: Calibration Loaded (v2)."));
    Serial.print(F("    R0_CO2: ")); Serial.println(calib.r0_co2);
    Serial.print(F("    PM25_Off: ")); Serial.println(calib.pm25_offset);
  } else {
    Serial.println(F(">>> EEPROM: No Valid Data or Old Version. Initializing Defaults..."));
    // First boot - restore defaults
    calib.r0_co = 10.0;
    calib.r0_co2 = 76.63;
    calib.r0_o3 = 100.0;
    calib.r0_nh3 = 50.0;
    calib.r0_so2 = 9.83;
    calib.r0_no2 = 2.5;
    calib.r0_tvoc = 0.6;
    calib.pm1_offset = 0.0;
    calib.pm10_offset = 0.0;
    calib.pm25_offset = 0.0;
    
    // Initialize Two-Point Defaults
    for(int i=0; i<NUM_SENSORS; i++) {
      calib.zero[i] = 0.0;
      calib.span[i] = 1.0;
    }
    
    calib.initialized = 0xA6;
    saveCalib();
  }
}

// ===== PROGRESS BAR HELPER =====
void drawProgressBar(int x, int y, int width, int height, int progress) {
  // progress: 0-100
  if (progress < 0) progress = 0;
  if (progress > 100) progress = 100;
  
  // Draw border
  display.drawRFrame(x, y, width, height, 3);
  
  // Fill interior
  int fillWidth = ((width - 4) * progress) / 100;
  if (fillWidth > 0) {
    display.drawRBox(x + 2, y + 2, fillWidth, height - 4, 2);
  }
}

// ===== BOOT SCREEN PHASES =====
void showBootPhase(const char* label, int progress) {
  display.setDrawColor(0); display.drawBox(0, 38, SCREEN_WIDTH, 26); display.setDrawColor(1);; // Clear lower area
  centerText(label, 40);
  drawProgressBar(0, 52, SCREEN_WIDTH, 10, progress);
  display.sendBuffer();
}

// ===== SETUP =====
void setup() {

  Serial.begin(9600);
  
  // Rotary Encoder
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  
  // Power up OLED first
  delay(250); 
  display.begin();
  display.setFontPosTop();
  display.clearBuffer();

  // ===== BOOT ANIMATION =====
  // Phase 1: Title reveal
  //display.setTextColor(...);
  centerText(F("AQI"), 5, 2);
  display.sendBuffer();
  delay(400);
  
  centerText(F("Monitoring System"), 25);
  display.sendBuffer();
  delay(300);
  
  centerText(F("v2.0 | WiFi"), 38);
  display.sendBuffer();
  delay(300);
  
  // Decorative line sweep animation
  for (int i = 0; i < SCREEN_WIDTH; i += 4) {
    display.drawHLine(0, 50, i);
    display.sendBuffer();
  }
  delay(500);
  
  // ===== SETUP PHASES WITH PROGRESS BAR =====
  display.clearBuffer();
  centerText(F("System Starting Up"), 5);
  display.drawHLine(0, 16, SCREEN_WIDTH);
  
  // Phase: Load config from EEPROM
  showBootPhase("Loading config...", 10);
  loadCalib();
  loadSensorConfig();
  loadUnitMode(); // Load Unit System Preference
  currentSendInterval = loadIntervalFromEEPROM();
  data_send.setdelay(currentSendInterval);
  showBootPhase("Config loaded!", 20);
  delay(300);
  
  // Phase: JioFi power-on (inline with progress)
  showBootPhase("Powering JioFi...", 25);
  pinMode(jiofi_switch, OUTPUT);
  digitalWrite(jiofi_switch, LOW);
  for (int i = 0; i < 10; i++) {
    showBootPhase("Powering JioFi...", 25 + i);
    delay(500);
  }
  digitalWrite(jiofi_switch, HIGH);
  
  // Phase: Wait for JioFi network
  showBootPhase("Waiting network...", 35);
  for (int i = 0; i < 25; i++) {
    showBootPhase("Waiting network...", 35 + i);
    delay(1000);
    Serial.print(F("."));
  }
  Serial.println();
  showBootPhase("Network ready!", 60);
  delay(300);
  
  // Phase: Initialize sensors
  showBootPhase("Init sensors...", 62);
  ESP32_SERIAL.begin(ESP32_BAUD);
  PM_SENSOR_SERIAL.begin(PM_SENSOR_BAUD);
  Serial.println(F("PM Sensor initialized on Serial3"));
  dht.begin();
  Serial.println(F("DHT22 sensor initialized on D4"));
  showBootPhase("Sensors ready!", 68);
  delay(300);
  
  // Phase: ESP32 startup
  showBootPhase("Starting ESP32...", 70);
  pinMode(ESP32_RESET_PIN, OUTPUT);
  digitalWrite(ESP32_RESET_PIN, LOW);
  Serial.println(F("Holding ESP32 in reset..."));
  delay(200);
  
  strcpy(sensorData.device_id, "DEVICE_001");
  
  Serial.println(F("Releasing ESP32 reset..."));
  digitalWrite(ESP32_RESET_PIN, HIGH);
  
  // Wait for ESP32 WiFi with animated progress (max 30s, 72->95%)
  showBootPhase("Connecting WiFi...", 72);
  Serial.println(F("Waiting for ESP32 to connect to WiFi..."));
  
  unsigned long startTime = millis();
  while (!esp32_ready && (millis() - startTime < 30000)) {
    checkESP32Messages();
    int elapsed = millis() - startTime;
    int progress = 72 + (elapsed * 23) / 30000;
    showBootPhase("Connecting WiFi...", progress);
    delay(100);
  }
  
  // Show connection result
  display.setDrawColor(0); display.drawBox(0, 20, SCREEN_WIDTH, 18); display.setDrawColor(1);;
  if (esp32_ready && wifi_status == 1) {
    centerText(F("WiFi: Connected!"), 24);
    Serial.println(F("ESP32 connected to WiFi!"));
  } else if (esp32_ready) {
    centerText(F("WiFi: Connecting..."), 24);
    Serial.println(F("ESP32 ready, WiFi pending..."));
  } else {
    centerText(F("ESP32: No Response"), 24);
    Serial.println(F("WARNING: ESP32 not responding!"));
    display.sendBuffer();
    delay(1000);
    hardResetESP32();
  }
  showBootPhase("Almost ready...", 95);
  delay(500);
  
  // ===== SENSOR WARMUP COUNTDOWN =====
  display.clearBuffer();
  centerText(F("Sensor Warmup"), 0);
  display.drawHLine(0, 10, SCREEN_WIDTH);
  centerText(F("MQ sensors heating"), 15);
  centerText(F("Please wait..."), 27);
  display.sendBuffer();
  
  int warmupSeconds = 30;
  for (int i = warmupSeconds; i > 0; i--) {
    display.setDrawColor(0); display.drawBox(0, 38, SCREEN_WIDTH, 26); display.setDrawColor(1);;
    
    // Time remaining - center dynamically
    String timeStr = String(i) + "s remaining";
    centerText(timeStr, 40);
    
    // Progress bar
    int progress = ((warmupSeconds - i) * 100) / warmupSeconds;
    drawProgressBar(0, 52, SCREEN_WIDTH, 10, progress);
    display.sendBuffer();
    delay(1000);
  }
  
  // ===== BOOT COMPLETE =====
  display.clearBuffer();
  centerText(F("Ready!"), 10, 2);
  centerText(F("System Online"), 35);
  drawProgressBar(0, 52, SCREEN_WIDTH, 10, 100);
  display.sendBuffer();
  delay(1500);
  display.clearBuffer();

  // Initialize MQ-7 heater cycling (start with heating phase)
  pinMode(MQ7_HEATER_PIN, OUTPUT);
  analogWrite(MQ7_HEATER_PIN, MQ7_PWM_HIGH); // Start at 5V
  mq7PhaseStart = millis();
  mq7Phase = MQ7_HEATING;
  Serial.println(F("MQ7: Heater cycling initialized (60s heat + 90s sense)"));

  // Initialize ABC window
  abcWindowStart = millis();
  Serial.println(F("ABC: Auto-baseline correction active (24h window)"));

  // === ENABLE HARDWARE WATCHDOG ===
  // 8-second timeout: if loop() freezes for >8s, Mega auto-reboots
  wdt_enable(WDTO_8S);
  Serial.println(F("Watchdog Timer enabled (8s timeout)"));
}

void loop() {
  wdt_reset(); // Feed the watchdog every loop iteration

  checkEncoder(); // Always check for menu open

  if (menuActive) {
    handleMenu();
    return; // BLOCK normal operation
  }

  // Check for messages from ESP32
  checkESP32Messages();

  // MQ-7 heater cycling (non-blocking)
  updateMQ7Heater();

  // Auto-Baseline Correction for VOC/NO2 (non-blocking)
  if (!menuActive) updateABC();

  // Update WiFi status icon (skip during OTA to preserve OTA screen)
  if (!otaInProgress) {
    drawWiFiIcon();
    display.sendBuffer();
  }

  // Skip all ESP32 communication and display updates during OTA
  if (!otaInProgress) {
    // Run timer callbacks
    esp32_status.update();

    if (wifi_status == 1) {
      oled_display.update();
      clear_area.update();
    }

    data_send.update();
  }
}

// Perform hardware reset of ESP32
void hardResetESP32() {
  wdt_reset(); // Feed watchdog before long operation

  // Show reset status on OLED
  display.setDrawColor(0); display.drawBox(0, 11, SCREEN_WIDTH, 54); display.setDrawColor(1);;
  centerText(F("WiFi Not Connected"), 22);
  centerText(F("Resetting ESP32..."), 32);
  display.sendBuffer();

  Serial.println(F("Performing Hard Reset of ESP32..."));
  digitalWrite(ESP32_RESET_PIN, LOW);  // Hold in reset
  delay(200);                          // Wait 200ms
  digitalWrite(ESP32_RESET_PIN, HIGH); // Release reset

  // Clear status variables
  esp32_ready = false;
  wifi_status = 0;

  // Wait for ESP32 to boot and connect to WiFi (max 30 seconds)
  display.clearBuffer();
  centerText(F("Waiting for ESP32"), 0);
  centerText(F("WiFi connection..."), 16);
  display.sendBuffer();
  Serial.println(F("Waiting for ESP32 to reconnect..."));

  unsigned long startTime = millis();
  while (!esp32_ready && (millis() - startTime < 30000)) {
    checkESP32Messages();
    delay(100);
    wdt_reset(); // Feed watchdog during wait

    // Show waiting animation
    static int dots = 0;
    display.setCursor(0, 32);
    //display.setTextColor(...);
    for (int i = 0; i < 3; i++) {
      display.print(i < dots ? "." : " ");
    }
    display.sendBuffer();
    dots = (dots + 1) % 4;
  }

  display.clearBuffer();

  if (esp32_ready && wifi_status == 1) {
    Serial.println(F("ESP32 reconnected to WiFi!"));
    centerText(F("WiFi: Reconnected!"), 20);
    display.sendBuffer();
    wdtDelay(1500);
  } else {
    Serial.println(F("ESP32 Reset Complete - WiFi not yet connected"));
    centerText(F("ESP32 Reset Done"), 20);
    display.sendBuffer();
    wdtDelay(1500);
  }
  display.clearBuffer();
}
