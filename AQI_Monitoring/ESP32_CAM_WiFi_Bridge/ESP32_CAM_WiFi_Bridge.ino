// ===== ESP32-CAM WiFi Bridge for AQI Monitoring System =====
// Receives sensor data from Arduino Mega via Serial and sends to server via WiFi
// Uses GPIO1/GPIO3 (USB Serial) for UART communication with Mega
// 
// ESP32-CAM Pin Mapping:
//   GPIO3 (RX) <- Mega TX2 (Pin 16) - WARNING: Lose USB Debugging on PC!
//   GPIO1 (TX) -> Mega RX2 (Pin 17) - WARNING: Lose USB Debugging on PC!
//   GPIO14     -> SD Card CLK (Freed up)
//   
// NOTE: SD Card uses 1-bit mode (SD_MMC) to save pins.
//       Pins used: 14 (CLK), 15 (CMD), 2 (D0)
//
// NOTE: GPIO4 (Flash LED) might blink during SD card access.

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>  // For parsing SSE commands
#include <esp_task_wdt.h>  // ESP32 Task Watchdog Timer
#include <HTTPUpdate.h>   // ESP32 OTA via HTTP
#include "FS.h"
#include "SD_MMC.h"

// ===== OTA Configuration =====
#define FIRMWARE_VERSION "1.1.1" // Bumped version
#define GITHUB_OWNER "aviralekka12"
#define GITHUB_REPO  "AQI_Monitoring"
#define OTA_CHECK_INTERVAL 120000UL  // Check every 120 sec (ms)

// ===== WiFi Configuration =====
const char* WIFI_SSID = "JioFi3";      // <-- Change this!
const char* WIFI_PASSWORD = "12345678@";  // <-- Change this!

// ===== Server Configuration =====
const char* SERVER_HOST = "www.airindex.online";
const int SERVER_PORT = 443;
const char* API_ENDPOINT = "/api/aqi/data";
const char* SSE_ENDPOINT = "/sse";
const char* DEVICE_ID = "DEVICE_001"; // Matching the Mega's ID

// ===== ESP32-CAM Serial Communication =====
// Using Standard Serial (USB) on GPIO3 (RX) and GPIO1 (TX)
#define MEGA_BAUD 115200

// ===== Status Variables =====
bool wifiConnected = false;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000;  // Check WiFi every 10 seconds
bool sdCardMounted = false;
long csvRowCount = 0; // Cached count of rows in data.csv

// ===== SSE Configuration =====
WiFiClientSecure sseClient;
unsigned long lastSSERetry = 0;
const unsigned long SSE_RETRY_INTERVAL = 5000;
unsigned long lastSSEStatus = 0;
const unsigned long SSE_STATUS_INTERVAL = 30000;  // Report status every 30 seconds
bool sseConnectedPrinted = false;  // Track if we printed the connection active message

// ===== Interval Management =====
long lastSentInterval = -1;  // Track last interval sent to Mega

// ===== OTA Variables =====
unsigned long lastOTACheck = 0;

// ===== Function Declarations =====
void connectWiFi();
bool sendToServer(String jsonPayload, long& extractedInterval);
void sendStatusToMega(String status);
void handleMegaMessages();
void processEvents();
void handleServerCommand(String jsonCommand);
void checkForOTAUpdate();
void logToSD(String message);
void logToCSV(String jsonData);
void countCSVRows();
void handleSDCommand(String command);
void formatSDCard();
void checkJioFiLTEStatus();

void setup() {
  // Initialize Serial for Mega Communication
  // Note: We use Serial for data now, not debug.
  Serial.begin(MEGA_BAUD);
  // Give Mega time to start
  delay(1000);

  // Initialize SD Card in 1-bit mode
  // true = 1-bit mode (frees up pins 4, 12, 13)
  if(!SD_MMC.begin("/sdcard", true)){
    sdCardMounted = false;
  } else {
    sdCardMounted = true;
    logToSD("===== SYSTEM RESTART =====");
    logToSD("SD Card Initialized");
    countCSVRows(); // Count existing rows on startup
  }

  logToSD("Firmware Version: " + String(FIRMWARE_VERSION));
  
  // Connect to WiFi
  connectWiFi();
  
  // Send ready status to Mega
  if (wifiConnected) {
    sendStatusToMega("WIFI_OK");
  }
  sendStatusToMega("READY");
  
  // Send firmware version to Mega for display
  sendStatusToMega("CMD:VERSION:" + String(FIRMWARE_VERSION));
  
  // === ENABLE ESP32 TASK WATCHDOG (30s timeout) ===
  esp_task_wdt_init(30, true);       // 30s timeout, panic (reboot) on trigger
  esp_task_wdt_add(NULL);            // Subscribe current task (loopTask)
  logToSD("Watchdog Enabled");
}

void loop() {
  esp_task_wdt_reset(); // Feed the watchdog every loop iteration

  // Check WiFi connection periodically
  if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      logToSD("WiFi Disconnected. Reconnecting...");
      wifiConnected = false;
      sendStatusToMega("WIFI_FAIL");
      connectWiFi();
      if (wifiConnected) {
        sendStatusToMega("WIFI_OK");
      }
    }
  }
  
  // Handle messages from Mega
  handleMegaMessages();
  
  // Handle Server-Sent Events
  processEvents();
  
  // Periodic OTA check (every 6 hours)
  if (wifiConnected && (millis() - lastOTACheck > OTA_CHECK_INTERVAL)) {
    lastOTACheck = millis();
    checkForOTAUpdate();
  }

  delay(10);  // Small delay to prevent watchdog issues
}


// Check JioFi LTE Status (looks for "Attached" on gateway page)
void checkJioFiLTEStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    logToSD("Checking JioFi LTE Status...");
    HTTPClient http;
    // JioFi gateway IP
    http.begin("http://192.168.225.1/");
    http.setTimeout(5000); 
    
    int httpCode = http.GET();
    
    if (httpCode > 0) {
      String payload = http.getString();
      // Search for "Attached" which indicates LTE connection is active
      if (payload.indexOf("Attached") > 0) {
        logToSD("LTE Status: Attached");
        sendStatusToMega("CMD:LTE_OK");
      } else {
        logToSD("LTE Status: Not Attached");
        sendStatusToMega("CMD:LTE_FAIL");
      }
    } else {
      logToSD("Error checking LTE: " + http.errorToString(httpCode));
      sendStatusToMega("CMD:LTE_FAIL"); // Treat connection error as fail
    }
    http.end();
  }
}

// Connect to WiFi with retry logic
void connectWiFi() {
  logToSD("Connecting to WiFi: " + String(WIFI_SSID));
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    logToSD("WiFi Connected. IP: " + WiFi.localIP().toString());
    
    // Check LTE Status immediately after connection
    checkJioFiLTEStatus();
    
  } else {
    wifiConnected = false;
    logToSD("WiFi Connection Failed");
  }
}

// Send JSON data to server via HTTPS
bool sendToServer(String jsonPayload, long& extractedInterval) {
  extractedInterval = -1; // Default to no interval found

  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    logToSD("Send Failed: No WiFi");
    return false;
  }
  
  logToSD("Sending Data (" + String(jsonPayload.length()) + " bytes)");
  
  WiFiClientSecure client;
  client.setInsecure();  // Skip certificate validation
  
  HTTPClient https;
  
  String url = "https://" + String(SERVER_HOST) + String(API_ENDPOINT);
  
  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/json");
    https.setTimeout(30000);  // 30 second timeout
    
    int httpCode = https.POST(jsonPayload);
    
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
        String response = https.getString();
        logToSD("Server Response: " + response);

        // Parse response to find send_interval
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, response);
        if (!error) {
          if (doc.containsKey("send_interval")) {
            long interval = doc["send_interval"];
            extractedInterval = interval;
            logToSD("New Interval: " + String(interval));
          }
        }

        https.end();
        return true;
      } else {
        logToSD("HTTP Error: " + String(httpCode));
      }
    } else {
      logToSD("Connection Error: " + https.errorToString(httpCode));
    }
    
    https.end();
  } else {
    logToSD("Failed to connect to server");
  }
  
  return false;
}

// Send status message to Mega
// This is the ONLY function that should write to Serial!
void sendStatusToMega(String status) {
  Serial.println(status);
}

// Handle incoming messages from Mega
void handleMegaMessages() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();
    
    if (message.length() == 0) return;
    
    // Check if it's a status request
    if (message == "STATUS") {
      if (wifiConnected && WiFi.status() == WL_CONNECTED) {
        sendStatusToMega("WIFI_OK");
      } else {
        sendStatusToMega("WIFI_FAIL");
      }
      return;
    }
    
    // Check if it is a command for the SD card
    if (message.startsWith("CMD:SD_")) {
      handleSDCommand(message);
      return;
    }
    
    // Check if it's a JSON payload (starts with '{')
    if (message.startsWith("{")) {
      logToSD("Processing JSON Payload");
      
      // Also save to CSV
      logToCSV(message);
      
      long newInterval = -1;
      // Send to server
      bool success = sendToServer(message, newInterval);
      
      if (success) {
        sendStatusToMega("OK");
        logToSD("Data Sent Successfully");
        
        // If server sent a new interval, check if it's different before forwarding
        if (newInterval > 0) {
          if (newInterval != lastSentInterval) {
            delay(100); // Small delay to ensure Mega processes the OK first
            String cmd = "CMD:SET_INTERVAL:" + String(newInterval);
            sendStatusToMega(cmd);
            lastSentInterval = newInterval; // Update tracking variable
            logToSD("Updated Interval: " + String(newInterval));
          }
        }
      } else {
        sendStatusToMega("FAIL");
        logToSD("Data Send Failed");
      }
    }
  }
}

// ===== SD CARD COMMANDS =====
void handleSDCommand(String command) {
  if (command == "CMD:SD_STAT") {
    if (!sdCardMounted) {
      sendStatusToMega("CMD:SD_STAT:FAIL");
      return;
    }
    uint64_t totalBytes = SD_MMC.totalBytes();
    uint64_t usedBytes = SD_MMC.usedBytes();
    uint64_t freeBytes = totalBytes - usedBytes;
    float freeMB = (float)freeBytes / (1024.0 * 1024.0);
    
    String response = "CMD:SD_STAT:" + String(csvRowCount) + "," + String(freeMB, 2);
    sendStatusToMega(response);
  }
  else if (command.startsWith("CMD:SD_DEL:")) {
    String filename = command.substring(11);
    if (!filename.startsWith("/")) filename = "/" + filename;
    
    if (SD_MMC.exists(filename)) {
      SD_MMC.remove(filename);
      logToSD("Deleted: " + filename);
      if (filename == "/data.csv") csvRowCount = 0; // Reset count
      sendStatusToMega("CMD:SD_DEL:OK");
    } else {
      sendStatusToMega("CMD:SD_DEL:FAIL");
    }
  }
  else if (command == "CMD:SD_FMT") {
    formatSDCard();
  }
}

void formatSDCard() {
  // ESP32 SD_MMC library doesn't expose a format() method easily.
  // We will simulate format by deleting known files.
  SD_MMC.remove("/log.txt");
  SD_MMC.remove("/data.csv");
  // Add other files if needed
  csvRowCount = 0;
  logToSD("SD Card Formatted (Simulated)");
  sendStatusToMega("CMD:SD_FMT:OK");
}

// ===== CSV LOGGING =====
void countCSVRows() {
  if (!sdCardMounted) return;
  if (!SD_MMC.exists("/data.csv")) {
    csvRowCount = 0;
    return;
  }
  
  File file = SD_MMC.open("/data.csv", FILE_READ);
  if (!file) return;
  
  long count = 0;
  while (file.available()) {
    char c = file.read();
    if (c == '\n') count++;
  }
  file.close();
  csvRowCount = count;
  logToSD("CSV Rows Counted: " + String(csvRowCount));
}

void logToCSV(String jsonData) {
  if (!sdCardMounted) return;
  
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    logToSD("CSV: JSON Parse Error");
    return;
  }
  
  File file = SD_MMC.open("/data.csv", FILE_APPEND);
  if (!file) {
    logToSD("CSV: File Open Error");
    return;
  }
  
  // If file is empty, write header
  if (file.size() == 0) {
    file.println("Timestamp,AQI,PM2.5,PM10,CO,CO2,O3,NH3,NO2,SO2,TVOC,Temp,Hum,Volt");
  }
  
  // "Timestamp" is just millis() for now since no RTC
  file.print(millis()); file.print(",");
  file.print(doc["aqi"].as<int>()); file.print(",");
  file.print(doc["pm2_5"].as<float>()); file.print(",");
  file.print(doc["pm10"].as<float>()); file.print(",");
  file.print(doc["co"].as<float>()); file.print(",");
  file.print(doc["co2"].as<float>()); file.print(",");
  file.print(doc["o3"].as<float>()); file.print(",");
  file.print(doc["nh3"].as<float>()); file.print(",");
  file.print(doc["no2"].as<float>()); file.print(",");
  file.print(doc["so2"].as<float>()); file.print(",");
  file.print(doc["tvoc"].as<float>()); file.print(",");
  file.print(doc["temperature"].as<float>()); file.print(",");
  file.print(doc["humidity"].as<float>()); file.print(",");
  file.print(doc["voltage"].as<float>());
  file.println();
  
  file.close();
  csvRowCount++;
}


// processEvents: Maintain SSE connection and handle incoming commands
void processEvents() {
  // Only proceed if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) return;

  // 1. Handle Reconnection
  if (!sseClient.connected()) {
    sseConnectedPrinted = false;  // Reset flag when disconnected
    if (millis() - lastSSERetry > SSE_RETRY_INTERVAL) {
      logToSD("SSE: Connecting...");
      sseClient.setInsecure(); // Allow self-signed/Cloudflare certs
      
      if (sseClient.connect(SERVER_HOST, SERVER_PORT)) {
        logToSD("SSE: Connected");
        
        // Construct GET request
        String request = String("GET ") + SSE_ENDPOINT + "?device_id=" + DEVICE_ID + " HTTP/1.1\r\n" +
                         "Host: " + SERVER_HOST + "\r\n" +
                         "Accept: text/event-stream\r\n" +
                         "Cache-Control: no-cache\r\n" +
                         "Connection: keep-alive\r\n" +
                         "\r\n";
                         
        sseClient.print(request);
        
        lastSSERetry = millis();
        lastSSEStatus = millis();  // Reset status timer on new connection
      } else {
        logToSD("SSE: Connection Failed");
        lastSSERetry = millis();
      }
    }
    return;
  }
  
  // Heartbeat logging
  if (!sseConnectedPrinted || (millis() - lastSSEStatus > SSE_STATUS_INTERVAL)) {
    if (!sseClient.available() || sseConnectedPrinted) {
      // logToSD("SSE: Active"); // Optional heartbeat log
      lastSSEStatus = millis();
      sseConnectedPrinted = true;
    }
  }

  // 2. Process Incoming Data
  while (sseClient.available()) {
    String line = sseClient.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) continue;  // Skip empty lines
    
    // Skip HTTP headers
    if (line.startsWith("HTTP/") || 
        (line.indexOf(':') > 0 && !line.startsWith("data:"))) {
      continue;
    }

    // Parse 'data:' lines
    if (line.startsWith("data:")) {
      String payload = line.substring(5); // Remove "data:"
      payload.trim();
      
      if (payload.length() > 0) {
        logToSD("SSE Command: " + payload);
        handleServerCommand(payload);
      }
    }
  }
}

// handleServerCommand: Parse and execute commands from server
void handleServerCommand(String jsonCommand) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonCommand);
  
  if (error) {
    logToSD("JSON Parse Error: " + String(error.c_str()));
    return;
  }
  
  const char* type = doc["type"];
  const char* command = doc["command"];
  
  if (type == nullptr || command == nullptr) return;
  
  if (strcmp(type, "command") == 0) {
    if (strcmp(command, "SET_INTERVAL") == 0) {
      long value = doc["value"];
      logToSD("Command: SET_INTERVAL " + String(value));
      
      String megaCommand = String("CMD:SET_INTERVAL:") + String(value);
      sendStatusToMega(megaCommand);
    } 
    else if (strcmp(command, "RESET") == 0) {
      logToSD("Command: RESET");
      sendStatusToMega("CMD:RESET");
    }
    else if (strcmp(command, "OTA_UPDATE") == 0) {
      logToSD("Command: OTA_UPDATE");
      checkForOTAUpdate();
    }
    else {
      logToSD("Unknown Command: " + String(command));
    }
  }
}

// ===== VERSION COMPARISON =====
int compareVersions(const char* v1, const char* v2) {
  int major1 = 0, minor1 = 0, patch1 = 0;
  int major2 = 0, minor2 = 0, patch2 = 0;
  sscanf(v1, "%d.%d.%d", &major1, &minor1, &patch1);
  sscanf(v2, "%d.%d.%d", &major2, &minor2, &patch2);
  
  if (major1 != major2) return (major1 > major2) ? 1 : -1;
  if (minor1 != minor2) return (minor1 > minor2) ? 1 : -1;
  if (patch1 != patch2) return (patch1 > patch2) ? 1 : -1;
  return 0;
}

// ===== GITHUB OTA UPDATE =====
void checkForOTAUpdate() {
  logToSD("Checking for OTA Update...");
  
  if (WiFi.status() != WL_CONNECTED) {
    logToSD("OTA: Skipped (No WiFi)");
    return;
  }
  
  sendStatusToMega("CMD:OTA_CHECK");
  
  WiFiClientSecure apiClient;
  apiClient.setInsecure(); 
  
  HTTPClient https;
  String apiUrl = "https://api.github.com/repos/" + String(GITHUB_OWNER) + "/" + String(GITHUB_REPO) + "/releases/latest";
  
  if (!https.begin(apiClient, apiUrl)) {
    logToSD("OTA: Failed to connect to GitHub");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  https.addHeader("User-Agent", "ESP32-AQI-Monitor");
  https.setTimeout(15000);
  
  esp_task_wdt_reset(); 
  int httpCode = https.GET();
  
  if (httpCode != 200) {
    logToSD("OTA: API Error " + String(httpCode));
    https.end();
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  String response = https.getString();
  https.end();
  
  esp_task_wdt_reset(); 
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, response);
  
  if (error) {
    logToSD("OTA: JSON Error");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  const char* tagName = doc["tag_name"];
  if (!tagName) {
    logToSD("OTA: No tag name");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  String remoteVersion = String(tagName);
  if (remoteVersion.startsWith("v") || remoteVersion.startsWith("V")) {
    remoteVersion = remoteVersion.substring(1);
  }
  
  logToSD("Remote Version: " + remoteVersion);
  
  if (compareVersions(remoteVersion.c_str(), FIRMWARE_VERSION) <= 0) {
    logToSD("OTA: Up to date");
    sendStatusToMega("CMD:OTA_UPTODATE");
    return;
  }
  
  logToSD("OTA: New update available!");
  
  String binUrl = "";
  JsonArray assets = doc["assets"];
  for (JsonObject asset : assets) {
    String name = asset["name"].as<String>();
    if (name.endsWith(".bin")) {
      binUrl = asset["browser_download_url"].as<String>();
      break;
    }
  }
  
  if (binUrl.length() == 0) {
    logToSD("OTA: No .bin found");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  logToSD("OTA: Starting Download...");
  sendStatusToMega("CMD:OTA_START");
  
  esp_task_wdt_delete(NULL); // Disable watchdog
  
  WiFiClientSecure otaClient;
  otaClient.setInsecure(); 
  
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(otaClient, binUrl);
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      logToSD("OTA: FAILED (" + httpUpdate.getLastErrorString() + ")");
      sendStatusToMega("CMD:OTA_FAIL");
      esp_task_wdt_add(NULL);
      break;
      
    case HTTP_UPDATE_NO_UPDATES:
      logToSD("OTA: No Updates");
      sendStatusToMega("CMD:OTA_UPTODATE");
      esp_task_wdt_add(NULL);
      break;
      
    case HTTP_UPDATE_OK:
      logToSD("OTA: Success! Rebooting...");
      sendStatusToMega("CMD:OTA_DONE");
      delay(1000);
      ESP.restart(); 
      break;
  }
}

// ===== SD CARD LOGGING =====
void logToSD(String message) {
  if (!sdCardMounted) return;
  
  File file = SD_MMC.open("/log.txt", FILE_APPEND);
  if(!file){
    return;
  }
  // Add timestamp if we had RTC, but just millis for now
  file.print("[");
  file.print(millis());
  file.print("] ");
  file.println(message);
  file.close();
}
