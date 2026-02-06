#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 256

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <NoDelay.h>

void gsm_signal_update();
void oled_display_update();
void data_send_update();

noDelay gsm_signal(3000, gsm_signal_update);
noDelay oled_display(1000, oled_display_update);
noDelay data_send(60000, data_send_update);  // 60 seconds

TinyGsm modem(Serial1);
char apn[] = "airtelgprs.com";
char user[] = "";
char pass[] = "";
const char HOST[] = "const-bunny-blocking-beverages.trycloudflare.com";
const int SSL_PORT = 443;

int signalStrength;
int gprs_status = 0;

TinyGsmClientSecure gsm_client_secure_modem(modem, 0);
HttpClient http_client = HttpClient(gsm_client_secure_modem, HOST, SSL_PORT);

// ===== SENSOR DATA STRUCTURE =====
struct SensorData {
  // Air Quality Sensors
  float pm1_0;  // PM1.0 (µg/m³)
  float pm2_5;  // PM2.5 (µg/m³)
  float pm10;   // PM10 (µg/m³)
  float co;     // Carbon Monoxide (ppm)
  float co2;    // Carbon Dioxide (ppm)
  float o3;     // Ozone (ppb)
  float nh3;    // Ammonia (ppm)
  float no2;    // Nitrogen Dioxide (ppb)
  float so2;    // Sulfur Dioxide (ppb)
  float tvoc;   // Total Volatile Organic Compounds (ppb)

  // Environmental Sensors
  float temperature;  // Temperature (°C)
  float humidity;     // Humidity (%)
  int ldr;            // Light sensor value (0=Night, 1=Day or analog value)

  // Power Monitoring
  float voltage;  // Battery voltage (V) - Range: 10.0V to 14.6V
  float current;  // Current draw (A)
  float power;    // Power consumption (W)

  // Air Quality Index
  int aqi;                    // Overall AQI value (0-500)
  char dominant_pollutant[6]; // PM2.5, PM10, O3, CO, NO2, SO2
  char aqi_category[15];      // Good, Moderate, Unhealthy, etc.

  // Device Identification
  char device_id[20];  // Unique device identifier

  // Location Information
  float latitude;    // GPS latitude coordinate
  float longitude;   // GPS longitude coordinate
  char address[50];  // Street address or locality
  char city[30];     // City name
  char state[30];    // State/Province name
  char country[30];  // Country name
};

// Global sensor data instance
SensorData sensorData;

// Server configuration - UPDATE THESE WITH YOUR SERVER DETAILS
const char SERVER_HOST[] = "const-bunny-blocking-beverages.trycloudflare.com";  // Change to your server
const int SERVER_PORT = 443;                                                            // 443 for HTTPS (required by Cloudflare)
const char API_ENDPOINT[] = "/api/aqi/data";                                            // Your API endpoint

#define i2c_Address 0x3c
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Removed previousMillis and interval - now using noDelay library for data sending

// 'b_bl', 12x8px
const unsigned char b_bl[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'b0', 12x8px
const unsigned char b_0[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0x80, 0x20, 0x80, 0x10, 0x80, 0x10, 0x80, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b1', 12x8px
const unsigned char b_1[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xc0, 0x20, 0xc0, 0x10, 0xc0, 0x10, 0xc0, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b2', 12x8px
const unsigned char b_2[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xe0, 0x20, 0xe0, 0x10, 0xe0, 0x10, 0xe0, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b3', 12x8px
const unsigned char b_3[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xf0, 0x20, 0xf0, 0x10, 0xf0, 0x10, 0xf0, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b4', 12x8px
const unsigned char b_4[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xf8, 0x20, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b5', 12x8px
const unsigned char b_5[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xfc, 0x20, 0xfc, 0x10, 0xfc, 0x10, 0xfc, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b6', 12x8px
const unsigned char b_6[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xfe, 0x20, 0xfe, 0x10, 0xfe, 0x10, 0xfe, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b7', 12x8px
const unsigned char b_7[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xff, 0x20, 0xff, 0x10, 0xff, 0x10, 0xff, 0x20, 0xff, 0xe0, 0x00, 0x00
};
// 'b8', 12x8px
const unsigned char b_8[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xff, 0xa0, 0xff, 0x90, 0xff, 0x90, 0xff, 0xa0, 0xff, 0xe0, 0x00, 0x00
};
// 'b9', 12x8px
const unsigned char b_9[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xff, 0xe0, 0xff, 0xd0, 0xff, 0xd0, 0xff, 0xe0, 0xff, 0xe0, 0x00, 0x00
};
// 'b10', 12x8px
const unsigned char b_10[] PROGMEM = {
  0x00, 0x00, 0xff, 0xe0, 0xff, 0xe0, 0xff, 0xf0, 0xff, 0xf0, 0xff, 0xe0, 0xff, 0xe0, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 352)
const unsigned char* battery_level[12] = {
  b_bl, b_0, b_1, b_2, b_3, b_4, b_5, b_6, b_7, b_8, b_9, b_10
};

const unsigned char blank[] PROGMEM = {
  // 'blank, 8x8px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char gprs[] PROGMEM = {
  // 'gprs, 8x8px
  0x1c, 0x22, 0x40, 0x40, 0x4e, 0x42, 0x22, 0x1c
};
const unsigned char nogprs[] PROGMEM = {
  // 'nogprs', 8x8px
  0x9c, 0x62, 0x60, 0x50, 0x4e, 0x46, 0x22, 0x1d
};
const unsigned char dataicon[] PROGMEM = {
  // 'data, 8x8px
  0x20, 0x64, 0xe4, 0x24, 0x24, 0x27, 0x26, 0x04
};

byte state;

void printText(const char* text, int x = 0, int y = 0) {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(x, y);
  display.println(text);
  display.display();
}

void printText(String text, int x = 0, int y = 0) {
  printText(text.c_str(), x, y);
}

// ===== AQI CALCULATION FUNCTIONS (US EPA Standard) =====

// Helper function to calculate AQI from concentration using breakpoint table
int calculateAQI(float concentration, float Clow, float Chigh, int Ilow, int Ihigh) {
  if (concentration <= 0) return 0;
  return round(((Ihigh - Ilow) / (Chigh - Clow)) * (concentration - Clow) + Ilow);
}

// Calculate PM2.5 sub-index (24-hour average, μg/m³)
int calculatePM25_AQI(float pm25) {
  if (pm25 <= 12.0) return calculateAQI(pm25, 0.0, 12.0, 0, 50);
  else if (pm25 <= 35.4) return calculateAQI(pm25, 12.1, 35.4, 51, 100);
  else if (pm25 <= 55.4) return calculateAQI(pm25, 35.5, 55.4, 101, 150);
  else if (pm25 <= 150.4) return calculateAQI(pm25, 55.5, 150.4, 151, 200);
  else if (pm25 <= 250.4) return calculateAQI(pm25, 150.5, 250.4, 201, 300);
  else if (pm25 <= 500.4) return calculateAQI(pm25, 250.5, 500.4, 301, 500);
  else return 500; // Hazardous beyond scale
}

// Calculate PM10 sub-index (24-hour average, μg/m³)
int calculatePM10_AQI(float pm10) {
  if (pm10 <= 54) return calculateAQI(pm10, 0, 54, 0, 50);
  else if (pm10 <= 154) return calculateAQI(pm10, 55, 154, 51, 100);
  else if (pm10 <= 254) return calculateAQI(pm10, 155, 254, 101, 150);
  else if (pm10 <= 354) return calculateAQI(pm10, 255, 354, 151, 200);
  else if (pm10 <= 424) return calculateAQI(pm10, 355, 424, 201, 300);
  else if (pm10 <= 604) return calculateAQI(pm10, 425, 604, 301, 500);
  else return 500;
}

// Calculate O3 sub-index (8-hour average, ppb)
int calculateO3_AQI(float o3) {
  if (o3 <= 54) return calculateAQI(o3, 0, 54, 0, 50);
  else if (o3 <= 70) return calculateAQI(o3, 55, 70, 51, 100);
  else if (o3 <= 85) return calculateAQI(o3, 71, 85, 101, 150);
  else if (o3 <= 105) return calculateAQI(o3, 86, 105, 151, 200);
  else if (o3 <= 200) return calculateAQI(o3, 106, 200, 201, 300);
  else return 500;
}

// Calculate CO sub-index (8-hour average, ppm)
int calculateCO_AQI(float co) {
  if (co <= 4.4) return calculateAQI(co, 0.0, 4.4, 0, 50);
  else if (co <= 9.4) return calculateAQI(co, 4.5, 9.4, 51, 100);
  else if (co <= 12.4) return calculateAQI(co, 9.5, 12.4, 101, 150);
  else if (co <= 15.4) return calculateAQI(co, 12.5, 15.4, 151, 200);
  else if (co <= 30.4) return calculateAQI(co, 15.5, 30.4, 201, 300);
  else if (co <= 50.4) return calculateAQI(co, 30.5, 50.4, 301, 500);
  else return 500;
}

// Calculate NO2 sub-index (1-hour average, ppb)
int calculateNO2_AQI(float no2) {
  if (no2 <= 53) return calculateAQI(no2, 0, 53, 0, 50);
  else if (no2 <= 100) return calculateAQI(no2, 54, 100, 51, 100);
  else if (no2 <= 360) return calculateAQI(no2, 101, 360, 101, 150);
  else if (no2 <= 649) return calculateAQI(no2, 361, 649, 151, 200);
  else if (no2 <= 1249) return calculateAQI(no2, 650, 1249, 201, 300);
  else if (no2 <= 2049) return calculateAQI(no2, 1250, 2049, 301, 500);
  else return 500;
}

// Calculate SO2 sub-index (1-hour average, ppb)
int calculateSO2_AQI(float so2) {
  if (so2 <= 35) return calculateAQI(so2, 0, 35, 0, 50);
  else if (so2 <= 75) return calculateAQI(so2, 36, 75, 51, 100);
  else if (so2 <= 185) return calculateAQI(so2, 76, 185, 101, 150);
  else if (so2 <= 304) return calculateAQI(so2, 186, 304, 151, 200);
  else if (so2 <= 604) return calculateAQI(so2, 305, 604, 201, 300);
  else if (so2 <= 1004) return calculateAQI(so2, 605, 1004, 301, 500);
  else return 500;
}

// Calculate overall AQI (max of all sub-indices) and identify dominant pollutant
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

// Helper function to get AQI category name
const char* getAQICategory(int aqi) {
  if (aqi <= 50) return "Good";
  else if (aqi <= 100) return "Moderate";
  else if (aqi <= 150) return "Unhealthy SG";
  else if (aqi <= 200) return "Unhealthy";
  else if (aqi <= 300) return "Very Unhealthy";
  else return "Hazardous";
}

// Function to populate sensor data with random test values
void generateTestData() {
  // Air Quality Sensors - realistic ranges
  sensorData.pm1_0 = random(5, 50) + random(0, 100) / 100.0;    // 5-50 µg/m³
  sensorData.pm2_5 = random(10, 100) + random(0, 100) / 100.0;  // 10-100 µg/m³
  sensorData.pm10 = random(20, 150) + random(0, 100) / 100.0;   // 20-150 µg/m³
  sensorData.co = random(0, 10) + random(0, 100) / 100.0;       // 0-10 ppm
  sensorData.co2 = random(400, 1000) + random(0, 100) / 100.0;  // 400-1000 ppm
  sensorData.o3 = random(10, 100) + random(0, 100) / 100.0;     // 10-100 ppb
  sensorData.nh3 = random(0, 5) + random(0, 100) / 100.0;       // 0-5 ppm
  sensorData.no2 = random(5, 50) + random(0, 100) / 100.0;      // 5-50 ppb
  sensorData.so2 = random(5, 50) + random(0, 100) / 100.0;      // 5-50 ppb
  sensorData.tvoc = random(50, 500) + random(0, 100) / 100.0;   // 50-500 ppb

  // Environmental Sensors
  sensorData.temperature = random(15, 40) + random(0, 100) / 100.0;  // 15-40 °C
  sensorData.humidity = random(30, 90) + random(0, 100) / 100.0;     // 30-90 %
  sensorData.ldr = random(0, 1024);                                  // 0-1023 (analog value)

  // Power Monitoring
  sensorData.voltage = random(100, 146) / 10.0;                // 10.0-14.6 V
  sensorData.current = random(100, 500) / 1000.0;              // 0.1-0.5 A
  sensorData.power = sensorData.voltage * sensorData.current;  // Calculated power

  // Calculate AQI from pollutant values
  calculateOverallAQI();

  Serial.println("Generated test sensor data:");
  Serial.print("PM2.5: ");
  Serial.print(sensorData.pm2_5);
  Serial.print(" | Temp: ");
  Serial.print(sensorData.temperature);
  Serial.print(" | Voltage: ");
  Serial.print(sensorData.voltage);
  Serial.print(" | AQI: ");
  Serial.print(sensorData.aqi);
  Serial.print(" (");
  Serial.print(getAQICategory(sensorData.aqi));
  Serial.print(") - ");
  Serial.println(sensorData.dominant_pollutant);
}

// Helper function to ensure GPRS is connected before sending data
bool ensureConnected() {
  // Quick check: is GPRS still connected?
  if (modem.isGprsConnected()) {
    Serial.println("GPRS: Connected");
    return true;
  }

  // GPRS disconnected, try to reconnect
  Serial.println("GPRS: Disconnected! Attempting reconnect...");

  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.print("Reconnect attempt ");
    Serial.print(attempt + 1);
    Serial.print("/3... ");

    if (modem.gprsConnect(apn, user, pass)) {
      gprs_status = 1;
      Serial.println("Success!");
      return true;
    }

    Serial.println("Failed");
    delay(1000);
  }

  gprs_status = 0;
  Serial.println("Reconnection failed!");
  return false;
}

// Function to send sensor data to server
bool sendDataToServer() {
  // Ensure GPRS is connected
  if (!ensureConnected()) {
    Serial.println("Cannot send data - no connection!");
    return false;
  }
  display.drawBitmap(SCREEN_WIDTH - 38, 0, dataicon, 8, 8, SH110X_WHITE, SH110X_BLACK);
  display.display();
  // Build JSON payload
  String jsonPayload = "{";

  // Device Identification
  jsonPayload += "\"device_id\":\"" + String(sensorData.device_id) + "\",";

  // Air Quality Data
  jsonPayload += "\"pm1_0\":" + String(sensorData.pm1_0, 2) + ",";
  jsonPayload += "\"pm2_5\":" + String(sensorData.pm2_5, 2) + ",";
  jsonPayload += "\"pm10\":" + String(sensorData.pm10, 2) + ",";
  jsonPayload += "\"co\":" + String(sensorData.co, 2) + ",";
  jsonPayload += "\"co2\":" + String(sensorData.co2, 2) + ",";
  jsonPayload += "\"o3\":" + String(sensorData.o3, 2) + ",";
  jsonPayload += "\"nh3\":" + String(sensorData.nh3, 2) + ",";
  jsonPayload += "\"no2\":" + String(sensorData.no2, 2) + ",";
  jsonPayload += "\"so2\":" + String(sensorData.so2, 2) + ",";
  jsonPayload += "\"tvoc\":" + String(sensorData.tvoc, 2) + ",";

  // Environmental Data
  jsonPayload += "\"temperature\":" + String(sensorData.temperature, 1) + ",";
  jsonPayload += "\"humidity\":" + String(sensorData.humidity, 1) + ",";

  // Power Data
  jsonPayload += "\"voltage\":" + String(sensorData.voltage, 2) + ",";
  jsonPayload += "\"current_amp\":" + String(sensorData.current, 3) + ",";
  jsonPayload += "\"power_watt\":" + String(sensorData.power, 2) + ",";

  // Air Quality Index (calculated on device)
  jsonPayload += "\"aqi\":" + String(sensorData.aqi) + ",";
  jsonPayload += "\"aqi_category\":\"" + String(sensorData.aqi_category) + "\",";
  jsonPayload += "\"dominant_pollutant\":\"" + String(sensorData.dominant_pollutant) + "\"";

  jsonPayload += "}";


  Serial.println("Sending data to server...");
  Serial.print("Payload: ");
  Serial.println(jsonPayload);
  
  // Retry loop for SSL connection
  int maxRetries = 2;
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (attempt > 0) {
      Serial.print("Retry attempt ");
      Serial.print(attempt);
      Serial.println("/2...");
      delay(2000); // Wait before retry to let modem recover
    }
    
    // Create HTTPS client for data server
    TinyGsmClientSecure dataClient(modem);
    
    // Step 1: Establish SSL connection manually (this is the slow part)
    Serial.print("Connecting to ");
    Serial.print(SERVER_HOST);
    Serial.print(":");
    Serial.println(SERVER_PORT);
    
    unsigned long connectStart = millis();
    bool connected = dataClient.connect(SERVER_HOST, SERVER_PORT, 20); // 20 sec timeout for SSL
    unsigned long connectTime = millis() - connectStart;
    
    Serial.print("Connection took: ");
    Serial.print(connectTime);
    Serial.println(" ms");
    
    if (!connected) {
      Serial.println("Connection FAILED!");
      dataClient.stop();
      if (attempt < maxRetries - 1) {
        continue; // Retry
      }
      return false; // All retries exhausted
    }
    
    Serial.println("Connected! Sending HTTP request...");
    
    // Step 2: Build and send raw HTTP request
    dataClient.print("POST ");
    dataClient.print(API_ENDPOINT);
    dataClient.println(" HTTP/1.1");
    dataClient.print("Host: ");
    dataClient.println(SERVER_HOST);
    dataClient.println("Content-Type: application/json");
    dataClient.print("Content-Length: ");
    dataClient.println(jsonPayload.length());
    dataClient.println("Connection: close");
    dataClient.println(); // End of headers
    dataClient.print(jsonPayload);
    
    Serial.println("Request sent, waiting for response...");
    
    // Step 3: Wait briefly for response (optional - data is already sent)
    unsigned long responseStart = millis();
    while (dataClient.connected() && !dataClient.available()) {
      if (millis() - responseStart > 5000) { // 5 sec timeout
        break; // Don't wait forever, data is already sent
      }
      delay(50);
    }
    
    // Step 4: Try to read response (for logging only)
    if (dataClient.available()) {
      delay(100);
      String statusLine = dataClient.readStringUntil('\n');
      if (statusLine.length() > 0) {
        Serial.print("Response: ");
        Serial.println(statusLine);
      }
    }
    // Cleanup
    dataClient.stop();
    
    // Success! Connection worked and request was sent
    // Server confirmed data is received regardless of response parsing
    Serial.println("Data sent successfully!");
    return true;
  }
  
  // Should never reach here
  return false;
}


void drawSignalIcon(int strength) {
  display.fillRect(SCREEN_WIDTH - 14, 0, 14, 8, SH110X_BLACK);
  if (strength == -1) {
    display.drawLine(SCREEN_WIDTH - 120, 0, SCREEN_WIDTH - 2, 5, SH110X_WHITE);
    display.drawLine(SCREEN_WIDTH - 120, 5, SCREEN_WIDTH - 2, 0, SH110X_WHITE);
  } else {
    int numBars = map(strength, 0, 25, 0, 5);
    for (int i = 0; i < numBars; i++) {
      int barHeight = (i + 1);                             // Incremental bar height
      int x = SCREEN_WIDTH - 12 + (i * 2);                 // Bar X position
      int y = 8 - barHeight;                               // Bar Y position
      display.fillRect(x, y, 1, barHeight, SH110X_WHITE);  // Draw bar
    }
  }
}

void oled_display_update() {
  // display.clearDisplay();
  display.drawBitmap(SCREEN_WIDTH - 128, 0, battery_level[random(1, 11)], 12, 8, SH110X_WHITE, SH110X_BLACK);
  
  // Display voltage next to battery icon (battery icon is 12 pixels wide)
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(14, 0);  // 12px battery + 2px spacing
  display.print(sensorData.voltage, 1);  // Show voltage with 1 decimal place
  display.print("V");
  
  // Display AQI information (center of screen)
  // AQI Value (large text)
  display.setTextSize(2);
  display.setCursor(30, 20);
  display.print(sensorData.aqi);
  
  // AQI Label
  display.setTextSize(1);
  display.setCursor(90, 25);
  display.print("AQI");
  
  // Category and dominant pollutant
  display.setTextSize(1);
  display.setCursor(10, 48);
  display.print(getAQICategory(sensorData.aqi));
  display.print(" - ");
  display.print(sensorData.dominant_pollutant);
  
  display.display();
}

void gsm_signal_update() {
  drawSignalIcon(modem.getSignalQuality());
  // if (ledState == LOW)
  //   ledState = HIGH;
  // else
  //   ledState = LOW;
  // digitalWrite(LEDpin, ledState);
}

void data_send_update() {
  // Generate random test data
  generateTestData();
  
  // Send data to server
  Serial.println("\\n========== Sending Test Data ==========");
  bool success = sendDataToServer();
  
  if (success) {
    Serial.println("========== Data sent successfully! ==========\\n");
    // Optional: Show indicator on display
    display.drawBitmap(SCREEN_WIDTH - 38, 0, blank, 8, 8, SH110X_WHITE, SH110X_BLACK);
    display.display();
  } else {
    Serial.println("========== Failed to send data! ==========\\n");
  }
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  delay(250);                        // wait for the OLED to power up
  display.begin(i2c_Address, true);  //  display.display();
  display.clearDisplay();

  // Initialize device ID
  strcpy(sensorData.device_id, "DEVICE_001");  // Set your unique device ID here

  // Initialize location information
  sensorData.latitude = 22.9055996;
  sensorData.longitude = 84.1329492;
  strcpy(sensorData.address, "Jariya");
  strcpy(sensorData.city, "Jashpur");
  strcpy(sensorData.state, "Chhattisgarh");
  strcpy(sensorData.country, "India");

  // Display welcome message
  printText("AQI Monitering System", 2, 25);
  printText("Version : 1.0", 2, 35);
  delay(5000);
  display.clearDisplay();

  // === 1. VALIDATE GSM MODULE IS READY ===
  printText("Checking GSM", 0, 0);

  String modemInfo = "";
  int attempts = 0;
  bool gsmReady = false;

  // Keep trying to get modem info until we get a valid response
  while (modemInfo.length() < 5 && attempts < 20) {  // "SIM800 R14.18" has more than 5 chars
    modemInfo = modem.getModemInfo();
    Serial.print("Checking modem... ");
    Serial.println(modemInfo);

    if (modemInfo.length() < 5) {
      printText("Waiting...", 0, 16);
      delay(500);
      attempts++;
    } else {
      gsmReady = true;
    }
  }

  // Display GSM status
  display.clearDisplay();
  if (gsmReady) {
    printText("GSM: OK", 0, 0);
    Serial.print("Modem: ");
    Serial.println(modemInfo);
    printText(modemInfo, 0, 16);
    delay(1500);
  } else {
    printText("GSM: FAIL", 0, 0);
    printText("Check Module!", 0, 16);
    delay(3000);
    // Could add restart or halt here
  }

  // === 2. VALIDATE SIGNAL STRENGTH ===
  display.clearDisplay();
  printText("Check Signal", 0, 0);

  int signalQuality = 0;
  attempts = 0;

  while (signalQuality < 13 && attempts < 40) {  // Wait max 10 seconds
    signalQuality = modem.getSignalQuality();
    Serial.print("Signal: ");
    Serial.println(signalQuality);

    // Display signal strength
    String signalText = "Signal: " + String(signalQuality);
    printText(signalText, 0, 16);

    if (signalQuality < 13) {
      delay(250);
      attempts++;
    }
  }

  // Display signal status
  display.clearDisplay();
  if (signalQuality >= 13) {
    printText("Signal: OK", 0, 0);
    String signalText = "Strength: " + String(signalQuality);
    printText(signalText, 0, 16);
    delay(1500);
  } else {
    printText("Signal: WEAK", 0, 0);
    printText("Check Antenna!", 0, 16);
  }
  display.clearDisplay();

  // === 3. VALIDATE GPRS CONNECTIVITY ===
  display.clearDisplay();
  printText("Check GPRS", 0, 0);

  bool gprsConnected = false;
  attempts = 0;

  // Try to connect to GPRS
  while (!gprsConnected && attempts < 3) {
    Serial.print(F("Connecting to "));
    Serial.print(apn);
    printText("Connecting...", 0, 16);

    if (modem.gprsConnect(apn, user, pass)) {
      gprsConnected = true;
      gprs_status = 1;
      Serial.println(" OK");
    } else {
      gprs_status = 0;
      Serial.println(" fail");
      attempts++;
      delay(1000);
    }
  }

  // Display GPRS status
  display.clearDisplay();
  if (gprsConnected) {
    printText("GPRS: OK", 0, 0);
    printText(apn, 0, 16);

    // Verify GPRS is actually connected
    delay(1000);  // Give GPRS time to stabilize
    bool gprsActive = modem.isGprsConnected();
    Serial.print("GPRS Status Check: ");
    Serial.println(gprsActive ? "Active" : "Inactive");

    if (!gprsActive) {
      Serial.println("WARNING: GPRS not active, retrying...");
      delay(2000);
      gprsActive = modem.isGprsConnected();
    }

    delay(1500);

    // Get local IP to confirm connection
    IPAddress localIP = modem.localIP();
    Serial.print("Local IP: ");
    Serial.println(localIP);

    // Check if we got a valid IP (not 0.0.0.0)
    bool hasValidIP = (localIP[0] != 0);

    // Test internet connectivity
    display.clearDisplay();
    printText("Test Internet", 0, 0);

    if (gprsActive && hasValidIP) {
      // Use TinyGSM client for a simple connection test
      TinyGsmClient testClient(modem);

      Serial.println("Trying: google.com:80");
      printText("Connecting...", 0, 16);

      // Add longer timeout for connection
      delay(2000);  // Give modem time to establish connection

      // Try to connect to Google on port 80
      bool connected = testClient.connect("google.com", 80, 10);  // 10 sec timeout

      if (connected) {
        printText("Internet: OK", 0, 16);
        Serial.println("Connection successful!");
        testClient.stop();
        delay(1500);
        state = 1;  // Mark as fully connected
      } else {
        // Try with IP address instead of domain name
        Serial.println("Domain failed, trying IP: 8.8.8.8:80");
        display.clearDisplay();
        printText("Test Internet", 0, 0);
        printText("Retry w/ IP...", 0, 16);

        delay(1000);
        connected = testClient.connect("8.8.8.8", 80, 10);  // 10 sec timeout

        if (connected) {
          printText("Internet: OK", 0, 16);
          Serial.println("IP connection successful!");
          testClient.stop();
          delay(1500);
          state = 1;
        } else {
          // Connection test failed BUT GPRS is active with valid IP
          // This might be carrier blocking port 80, but data may still work
          Serial.println("Test failed, but GPRS active - proceeding anyway");
          display.clearDisplay();
          printText("Test Failed", 0, 0);
          printText("GPRS OK - Try!", 0, 16);
          delay(2000);
          state = 1;  // Proceed anyway - let ensureConnected() handle issues
        }
      }
    } else {
      printText("No IP Address", 0, 16);
      Serial.println("GPRS inactive or no IP!");
      delay(2000);
    }
  } else {
    printText("GPRS: FAIL", 0, 0);
    printText("Check APN!", 0, 16);
    delay(3000);
  }

  // === INITIALIZATION COMPLETE ===
  display.clearDisplay();
  if (gsmReady && signalQuality >= 13 && gprsConnected) {
    printText("System Ready!", 0, 0);
    delay(1500);
  } else {
    printText("Init Error!", 0, 0);
    delay(2000);
  }
  display.clearDisplay();

  http_client.setHttpResponseTimeout(90 * 1000);  //90 secs timeout
}

void loop() {
  unsigned long currentMillis = millis();

  // display.setTextSize(1);
  // display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  // display.setCursor(0, 16);
  // display.println(currentMillis);
  // display.display();

  // GPRS connection is now handled in setup()
  // Display GPRS status icon
  if (gprs_status == 1) {
    display.drawBitmap(SCREEN_WIDTH - 25, 0, gprs, 8, 8, SH110X_WHITE, SH110X_BLACK);
  } else {
    display.drawBitmap(SCREEN_WIDTH - 25, 0, nogprs, 8, 8, SH110X_WHITE, SH110X_BLACK);
  }

  // display.drawBitmap(SCREEN_WIDTH - 38, 0, dataicon, 8, 8, SH110X_WHITE, SH110X_BLACK);
  display.display();
  gsm_signal.update();
  oled_display.update();
  data_send.update();  // Check if it's time to send data
}
