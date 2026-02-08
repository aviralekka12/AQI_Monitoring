// ===== AQI MONITORING SYSTEM - ESP32 WiFi Version =====
// Arduino Mega 2560 - Sensor Hub & Display
// Communicates with ESP32 via Serial2 for WiFi connectivity

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include <EEPROM.h>
#include <NoDelay.h>
#include <SPI.h>
#include <Wire.h>

#define jiofi_switch 46

// ===== SENSOR PIN DEFINITIONS =====
#define DHT_PIN 4
#define DHT_TYPE DHT22

#define MQ7_PIN A1   // CO sensor
#define MQ2_PIN A10  // SO2 sensor
#define MQ131_PIN A8 // O3 sensor
#define MQ137_PIN A6 // NH3 sensor
#define NO2_PIN A4   // Fermion MEMS NO2
#define MQ135_PIN A2 // CO2 sensor
#define VOC_PIN A0   // DFRobot MEMS VOC

// DHT Sensor object
// DHT Sensor object
DHT dht(DHT_PIN, DHT_TYPE);

// ===== CALIBRATION & EEPROM =====
// EEPROM Address Map
#define EEPROM_INTERVAL_ADDR 0     // 4 bytes
#define EEPROM_CALIB_ADDR 10       // Start of calibration data

// Measured Load Resistors (RL) in kOhms
#define RL_MQ7 0.694
#define RL_MQ2 0.794
#define RL_MQ131 0.698
#define RL_MQ137 0.783
#define RL_MQ135 0.793
#define RL_DEFAULT 10.0 // For others if needed

struct CalibData {
  float r0_co;    // MQ7
  float r0_co2;   // MQ135
  float r0_o3;    // MQ131
  float r0_nh3;   // MQ137
  float r0_so2;   // MQ2 (or dedicated)
  float r0_no2;   // Fermion NO2 (Baseline Offset or Sensitivity)
  float r0_tvoc;  // MEMS VOC (Baseline)
  uint8_t initialized; // Magic byte (0xA5)
};

CalibData calib = {
    10.0, 76.63, 100.0, 50.0, 9.83, 2.5, 0.6, 0}; // Default values

// ===== ROTARY ENCODER =====
#define ENC_SW 32
#define ENC_DT 34
#define ENC_CLK 36

int lastClk = HIGH;
unsigned long lastButtonPress = 0;
bool menuActive = false;


// Forward declarations
void esp32_status_update();
void oled_display_update();
void data_send_update();
void clear_area_update();

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
Adafruit_SH1106G display =
    Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== BITMAP ICONS =====

// 'b_bl', 12x8px
const unsigned char b_bl[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00};

// 'b0', 12x8px
const unsigned char b_0[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0x80, 0x20,
                                     0x80, 0x10, 0x80, 0x10, 0x80, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b1', 12x8px
const unsigned char b_1[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xc0, 0x20,
                                     0xc0, 0x10, 0xc0, 0x10, 0xc0, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b2', 12x8px
const unsigned char b_2[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xe0, 0x20,
                                     0xe0, 0x10, 0xe0, 0x10, 0xe0, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b3', 12x8px
const unsigned char b_3[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xf0, 0x20,
                                     0xf0, 0x10, 0xf0, 0x10, 0xf0, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b4', 12x8px
const unsigned char b_4[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xf8, 0x20,
                                     0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b5', 12x8px
const unsigned char b_5[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xfc, 0x20,
                                     0xfc, 0x10, 0xfc, 0x10, 0xfc, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b6', 12x8px
const unsigned char b_6[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xfe, 0x20,
                                     0xfe, 0x10, 0xfe, 0x10, 0xfe, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b7', 12x8px
const unsigned char b_7[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xff, 0x20,
                                     0xff, 0x10, 0xff, 0x10, 0xff, 0x20,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b8', 12x8px
const unsigned char b_8[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xff, 0xa0,
                                     0xff, 0x90, 0xff, 0x90, 0xff, 0xa0,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b9', 12x8px
const unsigned char b_9[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xff, 0xe0,
                                     0xff, 0xd0, 0xff, 0xd0, 0xff, 0xe0,
                                     0xff, 0xe0, 0x00, 0x00};
// 'b10', 12x8px
const unsigned char b_10[] PROGMEM = {0x00, 0x00, 0xff, 0xe0, 0xff, 0xe0,
                                      0xff, 0xf0, 0xff, 0xf0, 0xff, 0xe0,
                                      0xff, 0xe0, 0x00, 0x00};

// Array of all bitmaps for convenience. (Total bytes used to store images in
// PROGMEM = 352)
const unsigned char *battery_level[12] = {b_bl, b_0, b_1, b_2, b_3, b_4,
                                          b_5,  b_6, b_7, b_8, b_9, b_10};

const unsigned char blank[] PROGMEM = {
    // 'blank, 8x8px
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 'no signal', 11x8px
const unsigned char epd_bitmap_no_wifi[] PROGMEM = {
    0x90, 0x40, 0x60, 0x40, 0x61, 0x40, 0x91, 0x40,
    0x05, 0x40, 0x05, 0x40, 0x15, 0x40, 0x55, 0x40};
// 'signal', 11x8px
const unsigned char epd_bitmap_wifi[] PROGMEM = {
    0x00, 0x40, 0x00, 0x40, 0x01, 0x40, 0x01, 0x40,
    0x05, 0x40, 0x05, 0x40, 0x15, 0x40, 0x55, 0x40};

const unsigned char dataicon[] PROGMEM = {
    // 'data, 8x8px
    0x20, 0x64, 0xe4, 0x24, 0x24, 0x27, 0x26, 0x04};

byte state;

void printText(const char *text, int x = 0, int y = 0, int size = 1) {
  display.setTextSize(size);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(x, y);
  display.println(text);
  display.display();
}

void printText(String text, int x = 0, int y = 0, int size = 1) {
  printText(text.c_str(), x, y, size);
}

// ===== EEPROM FUNCTIONS =====

void saveCalib() {
  EEPROM.put(EEPROM_CALIB_ADDR, calib);
  Serial.println("Calibration data saved to EEPROM.");
}

void loadCalib() {
  CalibData stored;
  EEPROM.get(EEPROM_CALIB_ADDR, stored);
  if (stored.initialized == 0xA5) {
    calib = stored;
    Serial.println("Calibration loaded.");
  } else {
    Serial.println("No calibration found, using defaults.");
    calib.initialized = 0xA5;
    saveCalib(); // Save defaults
  }
  // Print loaded values for debug
  Serial.print("R0 CO: "); Serial.println(calib.r0_co);
  Serial.print("R0 CO2: "); Serial.println(calib.r0_co2);
}

// Save interval to EEPROM
void saveIntervalToEEPROM(unsigned long interval) {
  EEPROM.put(EEPROM_INTERVAL_ADDR, interval);
  Serial.print("Saved interval to EEPROM: ");
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

  Serial.print("Loaded interval from EEPROM: ");
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
  int pm25_aqi = calculatePM25_AQI(sensorData.pm2_5);
  int pm10_aqi = calculatePM10_AQI(sensorData.pm10);
  int o3_aqi = calculateO3_AQI(sensorData.o3);
  int co_aqi = calculateCO_AQI(sensorData.co);
  int no2_aqi = calculateNO2_AQI(sensorData.no2);
  int so2_aqi = calculateSO2_AQI(sensorData.so2);

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

// ===== GAS SENSOR READING FUNCTIONS =====

// Read MQ-7 CO sensor (ppm)
// Using datasheet-accurate parameters
// Read MQ-7 CO sensor (ppm)
// Read MQ-7 CO sensor (ppm)
float readCO() {
  int rawValue = analogRead(MQ7_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_co = rawVoltage;
  sm_v_co = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_co);
  
  if (sm_v_co < 0.1) sm_v_co = 0.1;

  float Rs = ((5.0 * RL_MQ7) / sm_v_co) - RL_MQ7;
  float ratio = Rs / calib.r0_co;

  // MQ-7 datasheet curve: ppm = 10^((log(Rs/R0) - 0.74) / -0.36)
  float ppm = pow(10, ((log10(ratio) - 0.74) / -0.36));

  float tempCompensation = 1.0 - ((sensorData.temperature - 20.0) * 0.004);
  float humidityCompensation = 1.0 - ((sensorData.humidity - 65.0) * 0.003);
  ppm = ppm * tempCompensation * humidityCompensation;

  return constrain(ppm, 0, 1000);
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
  float ratio = Rs / calib.r0_co2;

  // MQ135 datasheet: ppm = 116.602 * (ratio ^ -2.769)
  float ppm = 116.6020682 * pow(ratio, -2.769034857);

  float tempCompensation = 1.0 - ((sensorData.temperature - 20.0) * 0.005);
  float humidityCompensation = 1.0 - ((sensorData.humidity - 65.0) * 0.004);
  ppm = ppm * tempCompensation * humidityCompensation;

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
  float ratio = Rs / calib.r0_o3;

  // ppb = 10^((log(Rs/R0) + 0.92) / -0.44)
  float ppb = pow(10, ((log10(ratio) + 0.92) / -0.44));

  float tempCompensation = 1.0 - ((sensorData.temperature - 20.0) * 0.006);
  float humidityCompensation = 1.0 - ((sensorData.humidity - 65.0) * 0.003);
  ppb = ppb * tempCompensation * humidityCompensation;

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
  float ratio = Rs / calib.r0_nh3;

  // ppm = 50.0 * pow(ratio, -1.2) - calibrated to 50ppm point
  float ppm = 50.0 * pow(ratio, -1.2);

  float tempCompensation = 1.0 - ((sensorData.temperature - 20.0) * 0.005);
  float humidityCompensation = 1.0 - ((sensorData.humidity - 55.0) * 0.004);
  ppm = ppm * tempCompensation * humidityCompensation;

  return constrain(ppm, 0, 500);
}



// Read GM-102B MEMS NO2 sensor (ppm)
// Using datasheet specifications
// Read GM-102B MEMS NO2 sensor (ppm)
// Read GM-102B MEMS NO2 sensor (ppm)
float readNO2() {
  int rawValue = analogRead(NO2_PIN);
  float rawVoltage = rawValue * (5.0 / 1023.0);
  
  if (!smoothingInitialized) sm_v_no2 = rawVoltage;
  sm_v_no2 = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * sm_v_no2);

  // Baseline voltage from calibration
  float baselineVoltage = calib.r0_no2;
  float deltaV = sm_v_no2 - baselineVoltage;
  float ppm = deltaV * 2.0; // Sensitivity

  // Clamp - don't show negative if below baseline
  if (ppm < 0) ppm = 0;

  return constrain(ppm, 0, 10.0);
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
  float ratio = Rs / calib.r0_so2;

  // MQ2 curve
  float ppb = 50.0 * pow(ratio, -1.4);

  float tempCompensation = 1.0 - ((sensorData.temperature - 20.0) * 0.005);
  float humidityCompensation = 1.0 - ((sensorData.humidity - 65.0) * 0.003);
  ppb = ppb * tempCompensation * humidityCompensation;

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
  ppb = constrain(ppb, 0, 2000);
  
  // Outlier Rejection: Don't allow massive jumps in one frame
  // e.g. 10 -> 600 is likely noise. 
  // Max change per loop (approx 100ms?) = 50ppb
  float diff = ppb - lastValidTVOC;
  if (diff > 50) ppb = lastValidTVOC + 50;
  else if (diff < -50) ppb = lastValidTVOC - 50;
  
  lastValidTVOC = ppb;
  
  /* Debugging
  Serial.print("TVOC RawV:"); Serial.print(rawVoltage);
  Serial.print(" SmV:"); Serial.print(sm_v_tvoc);
  Serial.print(" Base:"); Serial.print(baselineVoltage);
  Serial.print(" PPB:"); Serial.println(ppb);
  */
  
  return ppb;
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



// Function to read all sensors and populate sensorData
void readAllSensors() {
  // Read real PM sensor data
  if (readPMData()) {
    sensorData.pm1_0 = pmData.pm1_0;
    sensorData.pm2_5 = pmData.pm2_5;
    sensorData.pm10 = pmData.pm10;
    Serial.println("--- PM Sensor Data (ug/m3) ---");
    Serial.print("PM 1.0: ");
    Serial.println(sensorData.pm1_0);
    Serial.print("PM 2.5: ");
    Serial.println(sensorData.pm2_5);
    Serial.print("PM 10:  ");
    Serial.println(sensorData.pm10);
  } else {
    Serial.println("PM Sensor: No data available, using last values");
  }

  // Read gas sensors
  sensorData.co = readCO();
  sensorData.co2 = readCO2();
  sensorData.o3 = readO3();
  sensorData.nh3 = readNH3();
  sensorData.no2 = readNO2();
  sensorData.so2 = readSO2();
  sensorData.tvoc = readTVOC();

  // Read DHT22 temperature and humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    sensorData.temperature = t;
    sensorData.humidity = h;
  } else {
    Serial.println("DHT22: Read failed, using last values");
  }

  // Power monitoring - these require specific sensors (INA219, voltage divider,
  // etc.) Set to 0 or implement actual sensor reading when available
  sensorData.voltage = 12.8; // Implement with voltage sensor when available
  sensorData.current = 2.5;  // Implement with current sensor when available
  sensorData.power = 32; // Will be voltage * current when sensors are available

  // Calculate AQI from pollutant values
  calculateOverallAQI();

  // Print sensor summary
  Serial.println("--- Gas Sensor Readings ---");
  Serial.print("CO: ");
  Serial.print(sensorData.co);
  Serial.println(" ppm");
  Serial.print("CO2: ");
  Serial.print(sensorData.co2);
  Serial.println(" ppm");
  Serial.print("O3: ");
  Serial.print(sensorData.o3);
  Serial.println(" ppb");
  Serial.print("NH3: ");
  Serial.print(sensorData.nh3);
  Serial.println(" ppm");
  Serial.print("NO2: ");
  Serial.print(sensorData.no2);
  Serial.println(" ppb");
  Serial.print("SO2: ");
  Serial.print(sensorData.so2);
  Serial.println(" ppb");
  Serial.print("TVOC: ");
  Serial.print(sensorData.tvoc);
  Serial.println(" ppb");
  Serial.println("--- Environmental ---");
  Serial.print("Temp: ");
  Serial.print(sensorData.temperature);
  Serial.println(" C");
  Serial.print("Humidity: ");
  Serial.print(sensorData.humidity);
  Serial.println(" %");
  Serial.println("--- AQI ---");
  Serial.print("AQI: ");
  Serial.print(sensorData.aqi);
  Serial.print(" (");
  Serial.print(sensorData.aqi_category);
  Serial.print(") - ");
  Serial.println(sensorData.dominant_pollutant);
}

// Alias for backward compatibility
void generateTestData() { readAllSensors(); }

// Build JSON payload from sensor data
String buildJsonPayload() {
  String json = "{";

  // Device Identification
  json += "\"device_id\":\"" + String(sensorData.device_id) + "\",";

  // Air Quality Data
  json += "\"pm1_0\":" + String(sensorData.pm1_0, 2) + ",";
  json += "\"pm2_5\":" + String(sensorData.pm2_5, 2) + ",";
  json += "\"pm10\":" + String(sensorData.pm10, 2) + ",";
  json += "\"co\":" + String(sensorData.co, 2) + ",";
  json += "\"co2\":" + String(sensorData.co2, 2) + ",";
  json += "\"o3\":" + String(sensorData.o3, 2) + ",";
  json += "\"nh3\":" + String(sensorData.nh3, 2) + ",";
  json += "\"no2\":" + String(sensorData.no2, 2) + ",";
  json += "\"so2\":" + String(sensorData.so2, 2) + ",";
  json += "\"tvoc\":" + String(sensorData.tvoc, 2) + ",";

  // Environmental Data
  json += "\"temperature\":" + String(sensorData.temperature, 1) + ",";
  json += "\"humidity\":" + String(sensorData.humidity, 1) + ",";

  // Power Data
  json += "\"voltage\":" + String(sensorData.voltage, 2) + ",";
  json += "\"current_amp\":" + String(sensorData.current, 3) + ",";
  json += "\"power_watt\":" + String(sensorData.power, 2) + ",";

  // Air Quality Index
  json += "\"aqi\":" + String(sensorData.aqi) + ",";
  json += "\"aqi_category\":\"" + String(sensorData.aqi_category) + "\",";
  json += "\"dominant_pollutant\":\"" + String(sensorData.dominant_pollutant) +
          "\"";

  json += "}";
  return json;
}

// Send data to ESP32 via Serial2
bool sendDataToESP32() {
  if (!esp32_ready) {
    Serial.println("ESP32 not ready!");
    return false;
  }

  // Show data sending icon
  display.drawBitmap(SCREEN_WIDTH - 38, 0, dataicon, 8, 8, SH110X_WHITE,
                     SH110X_BLACK);
  display.display();

  // Build and send JSON
  String jsonPayload = buildJsonPayload();

  Serial.println("Sending data to ESP32...");
  Serial.print("Payload: ");
  Serial.println(jsonPayload);

  // Send to ESP32 with newline terminator
  ESP32_SERIAL.println(jsonPayload);

  // Wait for response (timeout 10 seconds)
  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < 10000) {
    if (ESP32_SERIAL.available()) {
      response = ESP32_SERIAL.readStringUntil('\n');
      response.trim();
      break;
    }
    delay(50);
  }

  // Clear data icon
  display.drawBitmap(SCREEN_WIDTH - 38, 0, blank, 8, 8, SH110X_WHITE,
                     SH110X_BLACK);
  display.display();

  if (response == "OK") {
    Serial.println("ESP32: Data sent successfully!");
    return true;
  } else if (response == "FAIL") {
    Serial.println("ESP32: Failed to send data!");
    return false;
  } else {
    Serial.print("ESP32: Unknown response: ");
    Serial.println(response);
    return false;
  }
}

void checkESP32Messages() {
  while (ESP32_SERIAL.available()) {
    String message = ESP32_SERIAL.readStringUntil('\n');
    message.trim();

    Serial.print("ESP32 Message: ");
    Serial.println(message);

    // Handle status messages
    if (message == "WIFI_OK") {
      wifi_status = 1;
      esp32_ready = true;
      Serial.println("WiFi connected!");
    } else if (message == "WIFI_FAIL") {
      wifi_status = 0;
      Serial.println("WiFi disconnected!");
    } else if (message == "READY") {
      esp32_ready = true;
      Serial.println("ESP32 is ready!");

      // Clear OLED display
      display.fillRect(0, 11, SCREEN_WIDTH, 54, SH110X_BLACK);
      display.display();
    }
    // Handle SSE commands from server (via ESP32)
    else if (message.startsWith("CMD:")) {
      // Parse command format: CMD:COMMAND_NAME:VALUE
      int firstColon = message.indexOf(':');
      int secondColon = message.indexOf(':', firstColon + 1);
      
      // Handle LTE Status Commands
      if (message == "CMD:LTE_OK") {
        Serial.println("✅ LTE Status: Attached");
        display.clearDisplay();
        printText("WiFi Connected", 0, 0);
        printText("LTE Status:", 0, 20);
        printText("Attached", 0, 32, 1);
        display.display();
        delay(3000); // Show status for 3 seconds
        display.clearDisplay();
      } else if (message == "CMD:LTE_FAIL") {
        Serial.println("❌ LTE Status: Not Attached");
        display.clearDisplay();
        printText("WiFi Connected", 0, 0);
        printText("LTE Status:", 0, 20);
        printText("Not Attached", 0, 32); // Font size 1 to fit
        display.display();
        delay(3000); // Show status for 3 seconds
        display.clearDisplay();
      }

      else if (firstColon > 0) {
        String command = message.substring(firstColon + 1);

        if (secondColon > 0) {
          // Command with value
          String cmdName = message.substring(firstColon + 1, secondColon);
          String cmdValue = message.substring(secondColon + 1);

          if (cmdName == "SET_INTERVAL") {
            long newInterval = cmdValue.toInt();

            Serial.println("═══════════════════════════════════");
            Serial.print("⏰ Current Interval: ");
            Serial.print(currentSendInterval);
            Serial.print(" ms (");
            Serial.print(currentSendInterval / 1000);
            Serial.println(" seconds)");
            Serial.print("⏰ New Interval: ");
            Serial.print(newInterval);
            Serial.print(" ms (");
            Serial.print(newInterval / 1000);
            Serial.println(" seconds)");

            if (newInterval >= 5000) { // Minimum 5 seconds
              long oldInterval =
                  currentSendInterval; // Save old value for display

              data_send.setdelay(newInterval);
              currentSendInterval = newInterval; // Update tracking variable
              saveIntervalToEEPROM(currentSendInterval); // Save to EEPROM
              Serial.println("✅ SSE Command: Interval updated successfully!");
              Serial.println("═══════════════════════════════════");

              // Show feedback on OLED
              display.clearDisplay();
              display.setTextSize(1);
              display.setTextColor(SH110X_WHITE, SH110X_BLACK);
              display.setCursor(0, 0);
              display.print("Config Updated!");
              display.setCursor(0, 16);
              display.print("Old: ");
              display.print(oldInterval / 1000);
              display.print("s");
              display.setCursor(0, 28);
              display.print("New: ");
              display.print(newInterval / 1000);
              display.print("s");
              display.display();
              delay(3000);
              display.clearDisplay();
            } else {
              Serial.print("❌ Invalid interval: ");
              Serial.print(newInterval);
              Serial.println(" ms (minimum 5000ms)");
              Serial.println("═══════════════════════════════════");
            }
          }
        } else {
          // Command without value
          if (command == "RESET") {
            Serial.println("✅ SSE Command: Reset requested");

            // Show reset message
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SH110X_WHITE, SH110X_BLACK);
            display.setCursor(0, 20);
            display.print("Resetting...");
            display.display();
            delay(1000);

            // Perform soft reset (restart ESP32 connection)
            esp32_ready = false;
            wifi_status = 0;
            Serial.println("Device reset complete");
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
        Serial.print("Updated data send interval to: ");
        Serial.print(newInterval);
        Serial.println(" ms");
      }
    }
  }
}

// Draw WiFi signal icon
void drawWiFiIcon() {
  if (wifi_status == 1) {
    display.drawBitmap(SCREEN_WIDTH - 11, 0, epd_bitmap_wifi, 11, 8,
                       SH110X_WHITE, SH110X_BLACK);
  } else {
    display.drawBitmap(SCREEN_WIDTH - 11, 0, epd_bitmap_no_wifi, 11, 8,
                       SH110X_WHITE, SH110X_BLACK);
  }
}

void clear_area_update() {
  display.fillRect(0, 14, 74, 54, SH110X_BLACK);
  if (page == 1) {
    Serial.println("Page 1");
    page = 2;
  } else {
    Serial.println("Page 2");
    page = 1;
  }
}

void oled_display_update() {
  // Continuously read PM sensor data for real-time display
  if (readPMData()) {
    sensorData.pm1_0 = pmData.pm1_0;
    sensorData.pm2_5 = pmData.pm2_5;
    sensorData.pm10 = pmData.pm10;
  }

  // Battery icon - show empty battery (or connect voltage sensor to calculate
  // actual level)
  display.drawBitmap(SCREEN_WIDTH - 128, 0, battery_level[0], 12, 8,
                     SH110X_WHITE, SH110X_BLACK);

  // Note: To display actual voltage, connect a voltage sensor and update
  // sensorData.voltage Then uncomment below: display.setTextSize(1);
  // display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  // display.setCursor(14, 0);
  // display.print(sensorData.voltage, 1);
  // display.print("V");

  // Draw separator line
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);
  // Draw vertical separator line
  display.drawFastVLine(74, 10, 54, SH110X_WHITE);

  // Total 42 pixel use
  printText("AQI", 94, 16);
  if (sensorData.aqi < 10) {
    printText(String(sensorData.aqi), 96, 30, 2);
  }
  if (sensorData.aqi > 9 && sensorData.aqi < 100) {
    printText(String(sensorData.aqi), 90, 30, 2);
  } else if (sensorData.aqi > 100) {
    printText(String(sensorData.aqi), 84, 30, 2);
  }

  if (page == 1) {
    printText("PM2.5:", 0, 14);
    printText(String(sensorData.pm2_5), 37, 14);

    printText("PM10 :", 0, 24);
    printText(String(sensorData.pm10), 37, 24);

    printText("CO   :", 0, 34);
    printText(String(readCO()), 37, 34);

    printText("CO2  :", 0, 44);
    printText(String(readCO2()), 37, 44);

    printText("O3   :", 0, 54);
    printText(String(readO3()), 37, 54);
  }

  if (page == 2) {
    printText("NH3  :", 0, 14);
    printText(String(readNH3()), 37, 14);

    printText("NO2  :", 0, 24);
    printText(String(readNO2()), 37, 24);

    printText("SO2  :", 0, 34);
    printText(String(readSO2()), 37, 34);

    printText("TVOC :", 0, 44);
    printText(String(readTVOC()), 37, 44);

    printText("RH   :", 0, 54);
    printText(String(sensorData.humidity), 37, 54);

    // printText("TEMPERATURE:", 0, 74);
    // printText(String(sensorData.temperature), 37, 74);
  }
}

void esp32_status_update() {
  // Request status from ESP32
  ESP32_SERIAL.println("STATUS");
  checkESP32Messages();
  drawWiFiIcon();
  display.display();
}

void data_send_update() {
  // Generate random test data
  generateTestData();
  display.fillRect(84, 30, 120, 42, SH110X_BLACK);
  display.display();

  // Send data to ESP32
  Serial.println("\n========== Sending Test Data ==========");
  Serial.print("Current Send Interval: ");
  Serial.print(currentSendInterval);
  Serial.print(" ms (");
  Serial.print(currentSendInterval / 1000);
  Serial.println(" seconds)");

  bool success = sendDataToESP32();

  if (success) {
    Serial.println("========== Data sent successfully! ==========\n");
    esp32_fail_count = 0; // Reset watchdog counter
  } else {
    Serial.println("========== Failed to send data! ==========\n");
    esp32_fail_count++;
    Serial.print("Failure count: ");
    Serial.print(esp32_fail_count);
    Serial.println("/2");

    if (esp32_fail_count >= 2) {
      Serial.println("Watchdog: Too many failures! Triggering System Reset...");
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
const char* sensorNames[] = {"CO", "CO2", "O3", "NH3", "NO2", "SO2", "TVOC"};
const bool nativeIsPPB[] = {false, false, true, false, false, true, true}; // true=PPB, false=PPM


// Display Helper
void centerText(String text, int y, int size = 1) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

// Perform Calculation and Save
void performCalibration() {
  display.clearDisplay();
  centerText("Calibrating...", 20, 1);
  display.display();

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

  // Take Samples
  for(int i=0; i<samples; i++) {
    rawSum += analogRead(pin);
    delay(100);
  }
  float avgRaw = rawSum / (float)samples;
  float voltage = avgRaw * (5.0 / 1023.0);
  if(voltage < 0.01) voltage = 0.01;

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
    float exponent = -0.36 * log10(calcTarget) + 0.74;
    float expectedRatio = pow(10, exponent);
    newVal = rs / expectedRatio;
    calib.r0_co = newVal;
  }
  else if (selectedSensorIndex == 1) { // CO2 (MQ135)
    float rs = ((5.0 * rl) / voltage) - rl;
    float expectedRatio = pow((calcTarget / 116.602), (1.0 / -2.769));
    newVal = rs / expectedRatio;
    calib.r0_co2 = newVal;
  }
  else if (selectedSensorIndex == 2) { // O3 (MQ131)
     float rs = ((5.0 * rl) / voltage) - rl;
     float exponent = -0.44 * log10(calcTarget) - 0.92;
     float expectedRatio = pow(10, exponent);
     newVal = rs / expectedRatio;
     calib.r0_o3 = newVal;
  }
  else if (selectedSensorIndex == 3) { // NH3 (MQ137)
    float rs = ((5.0 * rl) / voltage) - rl;
    float expectedRatio = pow((calcTarget/50.0), (1.0/-1.2));
    newVal = rs / expectedRatio;
    calib.r0_nh3 = newVal;
  }
  else if (selectedSensorIndex == 4) { // NO2 (Fermion)
    newVal = voltage; // Baseline Voltage
    calib.r0_no2 = newVal;
  }
  else if (selectedSensorIndex == 5) { // SO2 (MQ2)
    float rs = ((5.0 * rl) / voltage) - rl;
    float expectedRatio = pow((calcTarget/50.0), (1.0/-1.4));
    newVal = rs / expectedRatio;
    calib.r0_so2 = newVal;
  }
  else if (selectedSensorIndex == 6) { // TVOC
    newVal = voltage; // Baseline
    calib.r0_tvoc = newVal;
  }

  saveCalib();
  delay(1000);
  menuState = 2; // Back to sensor select
  redraw = true;
}


void handleMenu() {
  if (!menuActive) return;

  if (redraw) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    int lineHeight = 10; // 8px font + 5px space
    int startY = 12; // Title height + space

    if (menuState == 1) { // Main Menu
      centerText("=== MENU ===", 0, 1);
      display.drawLine(0, 9, 128, 9, SH110X_WHITE);
      
      const char* mainItems[] = {"1. Calibrate", "2. Net Config", "3. Reset", "4. About", "5. Exit"};
      int totalItems = 5;
      maxSelection = totalItems - 1;

      // Scrolling Window (Show 4 lines max)
      int windowSize = 5;
      int startIdx = 0;
      if (menuSelection >= windowSize) startIdx = menuSelection - windowSize + 1;

      for(int i=0; i<windowSize; i++) {
         int idx = startIdx + i;
         if (idx < totalItems) {
             int y = startY + (i * lineHeight);
             display.setCursor(0, y);
             if(idx == menuSelection) display.print("> "); else display.print("  ");
             display.print(mainItems[idx]);
         }
      }
    }
    else if (menuState == 2) { // Calibration Select
      centerText("= SENSORS =", 0, 1);
      display.drawLine(0, 9, 128, 9, SH110X_WHITE);

      int totalItems = 8; // 7 Sensors + Back
      maxSelection = totalItems - 1;
      
      int windowSize = 5;
      int startIdx = 0;
      if (menuSelection >= windowSize) startIdx = menuSelection - windowSize + 1;

      for(int i=0; i<windowSize; i++) {
         int idx = startIdx + i;
         if (idx < totalItems) {
            int y = startY + (i * lineHeight);
            display.setCursor(0, y);
            if(idx == menuSelection) display.print("> "); else display.print("  ");
            
            if(idx < 7) display.print(sensorNames[idx]);
            else display.print("Back");
         }
      }
    }
    else if (menuState == 21) { // Sensor Options (New State)
       centerText("= OPTIONS =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       
       const char* optItems[] = {"1. Air (Ratio)", "2. Air (Target)", "3. Lab Calib", "4. Unit:", "5. Back"};
       maxSelection = 4;
       
       for(int i=0; i<5; i++) {
           int y = startY + (i * lineHeight);
           display.setCursor(0, y);
           if(i == menuSelection) display.print("> "); else display.print("  ");
           display.print(optItems[i]);
           if (i == 3) {
             display.print(displayInPPB ? " PPB" : " PPM");
           }
       }
    }

    else if (menuState == 3) { // Calibration Set PPM

      centerText("= SET TARGET =", 0, 1);
      display.drawLine(0, 9, 128, 9, SH110X_WHITE);
      
      display.setCursor(0, 15);
      display.print("Sensor: "); display.println(sensorNames[selectedSensorIndex]);

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
      }

      // Display Current
      display.setCursor(0, 25);
      display.print("Curr: "); 
      
      // Convert Current Reading for Display
      bool native = nativeIsPPB[selectedSensorIndex];
      float displayVal = currentVal;
      if (displayInPPB && !native) displayVal *= 1000.0;
      else if (!displayInPPB && native) displayVal /= 1000.0;
      
      display.print(displayVal, 1);
      display.println(displayInPPB ? " ppb" : " ppm");
      
      // Display Target
      display.setCursor(0, 35);
      display.print("Tgt:  "); 
      display.print(targetPPM, 1);
      display.println(displayInPPB ? " ppb" : " ppm");
      
      centerText("[Click for Options]", 55, 1);
      maxSelection = 10000;
    }
    else if (menuState == 31) { // Confirm Calibration Action
       centerText("= ACTION =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       
       const char* actItems[] = {"1. Save & Exit", "2. Continue Edit", "3. Exit (No Save)"};
       int totalItems = 3;
       maxSelection = totalItems - 1;
       
       for(int i=0; i<totalItems; i++) {
         int y = startY + (i * lineHeight);
         display.setCursor(0, y);
         if(i == menuSelection) display.print("> "); else display.print("  ");
         display.print(actItems[i]);
       }
    }




    else if (menuState == 5) { // Network Config Submenu
      centerText("= NET CONFIG =", 0, 1);
      display.drawLine(0, 9, 128, 9, SH110X_WHITE);

      const char* netItems[] = {"1. Status", "2. SSID", "3. Password", "4. Back"};
      int totalItems = 4;
      maxSelection = totalItems - 1;
      
      for(int i=0; i<totalItems; i++) {
        int y = startY + (i * lineHeight);
        display.setCursor(0, y);
        if(i == menuSelection) display.print("> "); else display.print("  ");
        display.print(netItems[i]);
      }
    }
    else if (menuState == 51) { // Net: Status
       centerText("= STATUS =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       centerText(wifi_status ? "Connected" : "Disconnected", 25, 1);
       centerText("[Click to Back]", 50, 1);
    }
    else if (menuState == 52) { // Net: SSID
       centerText("= SSID =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       centerText("SSID: JIOFI_xxxx", 20, 1);
       centerText("(View Only)", 35, 1);
       centerText("[Click to Back]", 50, 1);
    }
    else if (menuState == 53) { // Net: Password
       centerText("= PASSWORD =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       centerText("Pass: ********", 20, 1);
       centerText("(View Only)", 35, 1);
       centerText("[Click to Back]", 50, 1);
    }
    else if (menuState == 6) { // Reset
       centerText("= RESET? =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       
       int yNo = 25;
       int yYes = 40;
       
       display.setCursor(20, yNo);
       if(menuSelection==0) display.print("> NO"); else display.print("  NO");
       
       display.setCursor(20, yYes);
       if(menuSelection==1) display.print("> YES"); else display.print("  YES");
       maxSelection = 1;
    }
    else if (menuState == 7) { // About
       centerText("= ABOUT =", 0, 1);
       display.drawLine(0, 9, 128, 9, SH110X_WHITE);
       centerText("AQI Monitor v1.1", 20, 1);
       centerText("Dev: Mega + ESP32", 32, 1);
       centerText("[Click to Back]", 50, 1);
    }

    display.display();
    redraw = false;
  }
  
  checkEncoder();
}


void checkEncoder() {
  int clkState = digitalRead(ENC_CLK);
  
  if (clkState != lastClk && clkState == LOW) { // Falling edge
    float inc = 1.0;
    if (menuState == 3) {
       // Adjust increment based on unit and value magnitude
       if (displayInPPB) inc = (targetPPM < 100) ? 1.0 : 10.0;
       else inc = (targetPPM < 10) ? 0.1 : 1.0; 
    }
    
    if (digitalRead(ENC_DT) != clkState) { // CW
      if (menuState == 3) targetPPM += inc;
      else if (menuSelection < maxSelection) menuSelection++;
    } else { // CCW
      if (menuState == 3) targetPPM -= inc;
      else if (menuSelection > 0) menuSelection--;
    }
    if(targetPPM < 0) targetPPM = 0;
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
        if (menuSelection == 0) { menuState = 2; menuSelection = 0; } // Calib
        else if (menuSelection == 1) { menuState = 5; menuSelection = 0; } // Net Config
        else if (menuSelection == 2) { menuState = 6; menuSelection = 0; } // Reset
        else if (menuSelection == 3) { menuState = 7; } // About
        else if (menuSelection == 4) { 
           menuActive = false; 
           menuState = 0; 
           display.clearDisplay(); 
           esp32_fail_count = 0;
        } // Exit
      }
      else if (menuState == 2) { // Sensor Select
        if (menuSelection == 7) { menuState = 1; menuSelection = 0; }
        else {
           selectedSensorIndex = menuSelection;
           menuState = 21; // Go to Options
           menuSelection = 0;
           
           // Initialize Default Unit for this sensor
           displayInPPB = nativeIsPPB[selectedSensorIndex];
        }
      }
      else if (menuState == 21) { // Sensor Options
         if (menuSelection == 0) { // Clean Air Calibration (Ratio Method)
             display.clearDisplay();
             centerText("Calibrating...", 25, 1);
             display.display();

             // Read current voltage
             float voltage = 0;
             float rl = 0;
             switch(selectedSensorIndex) {
               case 0: voltage = analogRead(MQ7_PIN)*(5.0/1023.0); rl = RL_MQ7; break;
               case 1: voltage = analogRead(MQ135_PIN)*(5.0/1023.0); rl = RL_MQ135; break;
               case 2: voltage = analogRead(MQ131_PIN)*(5.0/1023.0); rl = RL_MQ131; break;
               case 3: voltage = analogRead(MQ137_PIN)*(5.0/1023.0); rl = RL_MQ137; break;
               case 4: voltage = analogRead(NO2_PIN)*(5.0/1023.0); break;
               case 5: voltage = analogRead(MQ2_PIN)*(5.0/1023.0); rl = RL_MQ2; break;
               case 6: voltage = analogRead(VOC_PIN)*(5.0/1023.0); break;
             }
             if (voltage < 0.1) voltage = 0.1;

             // Logic for Ratio Method
             if (selectedSensorIndex == 4 || selectedSensorIndex == 6) {
                // MEMS (Zero/Baseline) - Set current voltage as baseline
                if (selectedSensorIndex == 4) calib.r0_no2 = voltage;
                else calib.r0_tvoc = voltage;
             } else {
                // MQ Sensors - Use Fixed Clean Air Ratio (Rs_Air / R0)
                // R0 = Rs_Air / Ratio
                float rs = ((5.0 * rl) / voltage) - rl;
                float ratio = 1.0;
                
                if (selectedSensorIndex == 0) ratio = 27.0; // MQ7 (CO) Clean Air Ratio
                else if (selectedSensorIndex == 1) ratio = 3.6; // MQ135 (CO2) ~400ppm
                else if (selectedSensorIndex == 2) ratio = 15.0; // MQ131 (O3) Clean Air (Est High R)
                else if (selectedSensorIndex == 3) ratio = 10.0; // MQ137 (NH3) Clean Air
                else if (selectedSensorIndex == 5) ratio = 9.8;  // MQ2 (SO2/Smoke) Clean Air
                
                float newR0 = rs / ratio;
                
                if (selectedSensorIndex == 0) calib.r0_co = newR0;
                else if (selectedSensorIndex == 1) calib.r0_co2 = newR0;
                else if (selectedSensorIndex == 2) calib.r0_o3 = newR0;
                else if (selectedSensorIndex == 3) calib.r0_nh3 = newR0;
                else if (selectedSensorIndex == 5) calib.r0_so2 = newR0;
             }
             
             saveCalib();
             delay(1000);
             
             // Show Done
             display.clearDisplay();
             centerText("Done!", 20, 2);
             centerText("Ratio Set", 40, 1);
             display.display();
             delay(1500);
             
             menuState = 2; // Back to list
             menuSelection = 0;
         } 
         else if (menuSelection == 1) { // Clean Air (Target Method)
             if(selectedSensorIndex == 1) targetPPM = 400.0; // CO2
             else if (selectedSensorIndex == 2) targetPPM = 20.0; // O3 (ppb) typical
             else targetPPM = 0.0;

             // Match Display Unit
             if (displayInPPB && !nativeIsPPB[selectedSensorIndex]) targetPPM *= 1000; 
             else if (!displayInPPB && nativeIsPPB[selectedSensorIndex]) targetPPM /= 1000;
             
             menuState = 3; 
         }
         else if (menuSelection == 2) { // Lab Calib (Manual)
             targetPPM = 0.0;
             menuState = 3;
         }
         else if (menuSelection == 3) { // Unit Toggle
             displayInPPB = !displayInPPB;
             redraw = true; 
         }
         else if (menuSelection == 4) { // Back
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
         else if (menuSelection == 3) { menuState = 1; menuSelection = 1; } // Back
      }
      else if (menuState == 51 || menuState == 52 || menuState == 53) {
         menuState = 5; // Back to Net Menu
      }
      else if (menuState == 6) { 
         if (menuSelection == 1) { calib.initialized = 0; loadCalib(); }
         menuState = 1; menuSelection = 2;
      }
      else if (menuState == 7) { // About Back
         menuState = 1; menuSelection = 3;
      }
      redraw = true;
    }
  }
}


void connectJioFi() {
  // Turning on
  display.clearDisplay();
  printText("Turning on JioFi...", 0, 20, 1);
  Serial.println("Turning on JioFi...");
  
  pinMode(jiofi_switch, OUTPUT);
  digitalWrite(jiofi_switch, LOW);
  delay(5000);
  digitalWrite(jiofi_switch, HIGH);
  
  // Waiting for network
  display.clearDisplay();
  printText("Waiting for Network..", 0, 20, 1);
  Serial.println("Waiting for JioFi Network...");
  
  // 25 seconds wait for network connection
  for(int i=0; i<25; i++) {
     delay(1000);
     Serial.print(".");
  }
  Serial.println();
  
  display.clearDisplay();
  printText("JioFi Connected", 0, 20, 1);
  Serial.println("JioFi Connected");
  delay(2000);
  display.clearDisplay();
}

void setup() {
  Serial.begin(9600);
  
  // Rotary Encoder
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  
  // Power up OLED first
  delay(250); 
  display.begin(i2c_Address, true);
  display.clearDisplay();

  loadCalib(); // Load Sensor Calibration

  // Initialize JioFi and wait for network
  connectJioFi();

  // Load saved interval from EEPROM
  currentSendInterval = loadIntervalFromEEPROM();
  data_send.setdelay(currentSendInterval);

  ESP32_SERIAL.begin(ESP32_BAUD);         // Serial2 for ESP32 communication
  PM_SENSOR_SERIAL.begin(PM_SENSOR_BAUD); // Serial3 for PM sensor
  Serial.println("PM Sensor initialized on Serial3");

  // Initialize DHT22 sensor
  dht.begin();
  Serial.println("DHT22 sensor initialized on D4");

  // Configure ESP32 Reset Pin
  pinMode(ESP32_RESET_PIN, OUTPUT);
  digitalWrite(ESP32_RESET_PIN, LOW); // Hold ESP32 in reset initially
  Serial.println("Holding ESP32 in reset...");

  display.clearDisplay(); // clear again just in case
  
  // Initialize device ID

  // Initialize device ID
  strcpy(sensorData.device_id, "DEVICE_001");

  // Display welcome message
  printText("AQI Monitoring System", 2, 20);
  printText("ESP32 WiFi Version", 10, 35);
  delay(2000);

  // Enable ESP32 now that Mega is ready
  Serial.println("Releasing ESP32 reset...");
  digitalWrite(ESP32_RESET_PIN, HIGH);
  display.clearDisplay();

  // === WAIT FOR ESP32 TO CONNECT ===
  printText("Waiting for ESP32", 0, 0);
  printText("WiFi connection...", 0, 16);

  Serial.println("Waiting for ESP32 to connect to WiFi...");

  // Wait for ESP32 ready signal (max 30 seconds)
  unsigned long startTime = millis();
  while (!esp32_ready && (millis() - startTime < 30000)) {
    checkESP32Messages();
    delay(100);

    // Update waiting animation
    static int dots = 0;
    display.setCursor(0, 32);
    display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    for (int i = 0; i < 3; i++) {
      display.print(i < dots ? "." : " ");
    }
    display.display();
    dots = (dots + 1) % 4;
  }

  display.clearDisplay();

  if (esp32_ready && wifi_status == 1) {
    printText("WiFi: Connected!", 0, 0);
    Serial.println("ESP32 connected to WiFi!");
    display.fillRect(0, 11, SCREEN_WIDTH, 54, SH110X_BLACK);
    delay(1500);
  } else if (esp32_ready) {
    printText("ESP32: Ready", 0, 0);
    printText("WiFi: Connecting...", 0, 16);
    Serial.println("ESP32 ready, waiting for WiFi...");
    delay(1500);
  } else {
    printText("ESP32: No Response", 0, 0);
    printText("Resetting...", 0, 16);
    Serial.println("WARNING: ESP32 not responding! Triggering Reset...");
    delay(2000);
    hardResetESP32(); // Try to recover
  }

  // === INITIALIZATION COMPLETE ===
  display.clearDisplay();
  printText("System Ready!", 0, 0);
  delay(1500);
  display.clearDisplay();
}

void loop() {
  checkEncoder(); // Always check for menu open

  if (menuActive) {
    handleMenu();
    return; // BLOCK normal operation
  }

  // Check for messages from ESP32
  checkESP32Messages();

  // Update WiFi status icon
  drawWiFiIcon();
  display.display();

  // Run timer callbacks
  esp32_status.update();

  if (wifi_status == 1) {
    oled_display.update();
    clear_area.update();
  }

  data_send.update();
}

// Perform hardware reset of ESP32
void hardResetESP32() {
  // Show reset status on OLED
  display.fillRect(0, 11, SCREEN_WIDTH, 54, SH110X_BLACK);
  printText("Wifi Not Connected", 10, 22);
  printText("Resetting ESP32...", 10, 32);
  display.display();

  Serial.println("Performing Hard Reset of ESP32...");
  digitalWrite(ESP32_RESET_PIN, LOW);  // Hold in reset
  delay(200);                          // Wait 200ms
  digitalWrite(ESP32_RESET_PIN, HIGH); // Release reset
  // Wait for boot
  delay(1000);
  Serial.println("ESP32 Reset Complete");
  // Clear status variables
  esp32_ready = false;
  wifi_status = 0;
}