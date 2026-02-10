// ===== ESP32-CAM WiFi Bridge for AQI Monitoring System =====
// Receives sensor data from Arduino Mega via Serial and sends to server via WiFi
// Uses GPIO13/GPIO14 for UART communication with Mega (ESP32-CAM compatible pins)
// 
// ESP32-CAM Pin Mapping:
//   GPIO13 (RX) <- Mega TX2 (Pin 16)
//   GPIO14 (TX) -> Mega RX2 (Pin 17)
//   
// NOTE: GPIO12 is a strapping pin - DO NOT USE for TX (causes boot failure)
//
// NOTE: ESP32-CAM has limited GPIO pins due to camera/SD card usage
// GPIO1/GPIO3 are used for USB serial debugging

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>  // For parsing SSE commands
#include <HardwareSerial.h>
#include <esp_task_wdt.h>  // ESP32 Task Watchdog Timer
#include <HTTPUpdate.h>   // ESP32 OTA via HTTP

// ===== OTA Configuration =====
#define FIRMWARE_VERSION "1.0.0"
#define GITHUB_OWNER "aviralekka12"
#define GITHUB_REPO  "AQI_Monitoring"
#define OTA_CHECK_INTERVAL 120000UL  // Check every 120 sec (ms)

// ===== WiFi Configuration =====
// const char* WIFI_SSID = "AirFiber";      // <-- Change this!
// const char* WIFI_PASSWORD = "6268123511";  // <-- Change this!

const char* WIFI_SSID = "JioFi3";      // <-- Change this!
const char* WIFI_PASSWORD = "12345678@";  // <-- Change this!

// ===== Server Configuration =====
const char* SERVER_HOST = "www.airindex.online";
const int SERVER_PORT = 443;
const char* API_ENDPOINT = "/api/aqi/data";
const char* SSE_ENDPOINT = "/sse";
const char* DEVICE_ID = "DEVICE_001"; // Matching the Mega's ID

// ===== ESP32-CAM Serial Communication =====
// Using HardwareSerial on GPIO13 (RX) and GPIO12 (TX)
// These pins are safe to use on ESP32-CAM when not using SD card in 4-bit mode
HardwareSerial MegaSerial(2);  // Use UART2

#define MEGA_BAUD 115200
#define MEGA_RX_PIN 13  // ESP32-CAM RX <- Mega TX2 (Pin 16)
#define MEGA_TX_PIN 14  // ESP32-CAM TX -> Mega RX2 (Pin 17) - GPIO12 causes boot issues!


// ===== Status Variables =====
bool wifiConnected = false;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000;  // Check WiFi every 10 seconds

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


void setup() {
  // Debug serial (USB) - uses GPIO1/GPIO3
  Serial.begin(115200);
  Serial.println("\n\n===== ESP32-CAM WiFi Bridge =====");
  Serial.print("Firmware Version: v");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Starting up...");
  
  
  // Serial for Mega communication using GPIO13 (RX) and GPIO12 (TX)
  MegaSerial.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  Serial.println("UART2 initialized for Mega communication");
  Serial.print("  RX Pin: GPIO");
  Serial.println(MEGA_RX_PIN);
  Serial.print("  TX Pin: GPIO");
  Serial.println(MEGA_TX_PIN);
  
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
  // Longer than Mega's 8s because of network operations (HTTPS, SSE)
  esp_task_wdt_init(30, true);       // 30s timeout, panic (reboot) on trigger
  esp_task_wdt_add(NULL);            // Subscribe current task (loopTask)
  Serial.println("ESP32 Watchdog Timer enabled (30s timeout)");
  
  Serial.println("ESP32-CAM Bridge Ready!");
}

void loop() {
  esp_task_wdt_reset(); // Feed the watchdog every loop iteration

  // Check WiFi connection periodically
  if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
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
    Serial.println("Checking JioFi LTE Status...");
    HTTPClient http;
    // JioFi gateway IP
    http.begin("http://192.168.225.1/");
    http.setTimeout(5000); 
    
    int httpCode = http.GET();
    
    if (httpCode > 0) {
      String payload = http.getString();
      // Search for "Attached" which indicates LTE connection is active
      // Based on user image, "LTE Status\nConnection Status:\nAttached"
      if (payload.indexOf("Attached") > 0) {
        Serial.println("LTE Status: Attached");
        sendStatusToMega("CMD:LTE_OK");
      } else {
        Serial.println("LTE Status: Not Attached");
        sendStatusToMega("CMD:LTE_FAIL");
      }
    } else {
      Serial.print("Error checking LTE status: ");
      Serial.println(http.errorToString(httpCode));
      sendStatusToMega("CMD:LTE_FAIL"); // Treat connection error as fail
    }
    http.end();
  }
}

// Connect to WiFi with retry logic
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");

    attempts++;
  }
  
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // Check LTE Status immediately after connection
    checkJioFiLTEStatus();
    
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi connection failed!");
  }
}

// Send JSON data to server via HTTPS
bool sendToServer(String jsonPayload, long& extractedInterval) {
  extractedInterval = -1; // Default to no interval found

  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi connection!");
    return false;
  }
  
  
  Serial.println("\n--- Sending to Server ---");
  Serial.print("Host: ");
  Serial.println(SERVER_HOST);
  Serial.print("Payload size: ");
  Serial.println(jsonPayload.length());
  
  WiFiClientSecure client;
  client.setInsecure();  // Skip certificate validation
  
  HTTPClient https;
  
  String url = "https://" + String(SERVER_HOST) + String(API_ENDPOINT);
  Serial.print("URL: ");
  Serial.println(url);
  
  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/json");
    https.setTimeout(30000);  // 30 second timeout
    
    Serial.println("Sending POST request...");
    int httpCode = https.POST(jsonPayload);
    
    Serial.print("HTTP Response Code: ");
    Serial.println(httpCode);
    
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
        String response = https.getString();
        Serial.print("Response: ");
        Serial.println(response);

        // Parse response to find send_interval
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, response);
        if (!error) {
          if (doc.containsKey("send_interval")) {
            long interval = doc["send_interval"];
            Serial.print("Found new interval in response: ");
            Serial.println(interval);
            extractedInterval = interval;
          }
        } else {
           Serial.print("JSON Parse Failed: ");
           Serial.println(error.c_str());
        }

        https.end();
        return true;
      } else {
        Serial.print("HTTP Error: ");
        Serial.println(httpCode);
      }
    } else {
      Serial.print("Connection Error: ");
      Serial.println(https.errorToString(httpCode));
    }
    
    https.end();
  } else {
    Serial.println("Failed to connect to server!");
  }
  
  return false;
}

// Send status message to Mega
void sendStatusToMega(String status) {
  MegaSerial.println(status);
  Serial.print("-> Mega: ");
  Serial.println(status);
}

// Handle incoming messages from Mega
void handleMegaMessages() {
  if (MegaSerial.available()) {
    String message = MegaSerial.readStringUntil('\n');
    message.trim();
    
    if (message.length() == 0) return;
    
    Serial.print("<- Mega: ");
    Serial.println(message);
    
    // Check if it's a status request
    if (message == "STATUS") {
      if (wifiConnected && WiFi.status() == WL_CONNECTED) {
        sendStatusToMega("WIFI_OK");
      } else {
        sendStatusToMega("WIFI_FAIL");
      }
      return;
    }
    
    // Check if it's a JSON payload (starts with '{')
    if (message.startsWith("{")) {
      Serial.println("Received JSON data from Mega");
      Serial.print("Payload: ");
      Serial.println(message);
      
      long newInterval = -1;
      // Send to server
      bool success = sendToServer(message, newInterval);
      
      if (success) {
        sendStatusToMega("OK");
        Serial.println("Data sent successfully!");
        
        // If server sent a new interval, check if it's different before forwarding
        if (newInterval > 0) {
          if (newInterval != lastSentInterval) {
            delay(100); // Small delay to ensure Mega processes the OK first
            String cmd = "CMD:SET_INTERVAL:" + String(newInterval);
            sendStatusToMega(cmd);
            lastSentInterval = newInterval; // Update tracking variable
            Serial.print("Forwarded new interval to Mega: ");
            Serial.println(newInterval);
          } else {
            Serial.print("Interval unchanged (");
            Serial.print(newInterval);
            Serial.println("), skipping update");
          }
        }
      } else {
        sendStatusToMega("FAIL");
        Serial.println("Failed to send data!");
      }
    }
  }
}

// processEvents: Maintain SSE connection and handle incoming commands
void processEvents() {
  // Only proceed if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) return;

  // 1. Handle Reconnection
  if (!sseClient.connected()) {
    sseConnectedPrinted = false;  // Reset flag when disconnected
    if (millis() - lastSSERetry > SSE_RETRY_INTERVAL) {
      Serial.println("SSE: Attempting connection...");
      sseClient.setInsecure(); // Allow self-signed/Cloudflare certs
      
      if (sseClient.connect(SERVER_HOST, SERVER_PORT)) {
        Serial.println("SSE: Connected!");
        
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
        Serial.println("SSE: Connection failed");
        lastSSERetry = millis();
      }
    }
    return;
  }
  
  // Print "Connection active" once after headers are processed, then every 30 seconds
  if (!sseConnectedPrinted || (millis() - lastSSEStatus > SSE_STATUS_INTERVAL)) {
    // Only print if we've processed HTTP headers (no more header lines incoming)
    if (!sseClient.available() || sseConnectedPrinted) {
      Serial.println("✅ SSE: Connection active, listening for commands...");
      lastSSEStatus = millis();
      sseConnectedPrinted = true;
    }
  }

  // 2. Process Incoming Data - silently process headers, only log data lines
  while (sseClient.available()) {
    String line = sseClient.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) continue;  // Skip empty lines
    
    // Skip HTTP headers (they start with HTTP/, or contain ":")
    if (line.startsWith("HTTP/") || 
        (line.indexOf(':') > 0 && !line.startsWith("data:"))) {
      // Silently skip headers - don't spam serial
      continue;
    }

    // Parse 'data:' lines - these are actual SSE events
    if (line.startsWith("data:")) {
      String payload = line.substring(5); // Remove "data:"
      payload.trim();
      
      if (payload.length() > 0) {
        Serial.println("🎯 SSE Command Detected!");
        Serial.print("📄 Payload: ");
        Serial.println(payload);
        
        // Handle command with JSON parsing
        handleServerCommand(payload);
      }
    }
  }
}

// handleServerCommand: Parse and execute commands from server
void handleServerCommand(String jsonCommand) {
  // Allocate JSON document
  StaticJsonDocument<512> doc;
  
  // Deserialize JSON
  DeserializationError error = deserializeJson(doc, jsonCommand);
  
  if (error) {
    Serial.print("JSON Parse Error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Extract command details
  const char* type = doc["type"];
  const char* command = doc["command"];
  
  if (type == nullptr || command == nullptr) {
    Serial.println("Invalid command format (missing type or command)");
    return;
  }
  
  Serial.print("Command Type: ");
  Serial.println(type);
  Serial.print("Command: ");
  Serial.println(command);
  
  // Handle different command types
  if (strcmp(type, "command") == 0) {
    if (strcmp(command, "SET_INTERVAL") == 0) {
      long value = doc["value"];
      Serial.print("Setting interval to: ");
      Serial.print(value);
      Serial.println(" ms");
      
      // Forward to Mega as a formatted command
      String megaCommand = String("CMD:SET_INTERVAL:") + String(value);
      MegaSerial.println(megaCommand);
      Serial.print("-> Mega: ");
      Serial.println(megaCommand);
    } 
    else if (strcmp(command, "RESET") == 0) {
      Serial.println("Received RESET command");
      MegaSerial.println("CMD:RESET");
      Serial.println("-> Mega: CMD:RESET");
    }
    else if (strcmp(command, "OTA_UPDATE") == 0) {
      Serial.println("Received OTA_UPDATE command from server");
      checkForOTAUpdate();
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
}

// ===== VERSION COMPARISON =====
// Returns: 1 if v1 > v2, -1 if v1 < v2, 0 if equal
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
  Serial.println("\n===== OTA Update Check =====");
  Serial.print("Current firmware: v");
  Serial.println(FIRMWARE_VERSION);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("OTA: No WiFi, skipping");
    return;
  }
  
  // Notify Mega
  sendStatusToMega("CMD:OTA_CHECK");
  
  // Step 1: Query GitHub API for latest release
  WiFiClientSecure apiClient;
  apiClient.setInsecure(); // Skip cert validation for GitHub API
  
  HTTPClient https;
  String apiUrl = "https://api.github.com/repos/" + String(GITHUB_OWNER) + "/" + String(GITHUB_REPO) + "/releases/latest";
  
  Serial.print("Checking: ");
  Serial.println(apiUrl);
  
  if (!https.begin(apiClient, apiUrl)) {
    Serial.println("OTA: Failed to connect to GitHub API");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  https.addHeader("User-Agent", "ESP32-AQI-Monitor");
  https.setTimeout(15000);
  
  esp_task_wdt_reset(); // Feed watchdog before network call
  int httpCode = https.GET();
  
  if (httpCode != 200) {
    Serial.print("OTA: GitHub API returned: ");
    Serial.println(httpCode);
    https.end();
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  // Step 2: Parse response
  String response = https.getString();
  https.end();
  
  esp_task_wdt_reset(); // Feed watchdog after download
  
  // Parse JSON - need larger doc for GitHub API response
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, response);
  
  if (error) {
    Serial.print("OTA: JSON parse error: ");
    Serial.println(error.c_str());
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  // Extract version from tag_name (e.g., "v1.1.0" -> "1.1.0")
  const char* tagName = doc["tag_name"];
  if (!tagName) {
    Serial.println("OTA: No tag_name in release");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  // Strip leading 'v' if present
  String remoteVersion = String(tagName);
  if (remoteVersion.startsWith("v") || remoteVersion.startsWith("V")) {
    remoteVersion = remoteVersion.substring(1);
  }
  
  Serial.print("Latest version: v");
  Serial.println(remoteVersion);
  
  // Step 3: Compare versions
  if (compareVersions(remoteVersion.c_str(), FIRMWARE_VERSION) <= 0) {
    Serial.println("OTA: Firmware is up to date!");
    sendStatusToMega("CMD:OTA_UPTODATE");
    return;
  }
  
  Serial.println("OTA: New version available!");
  
  // Step 4: Find the .bin asset download URL
  String binUrl = "";
  JsonArray assets = doc["assets"];
  for (JsonObject asset : assets) {
    String name = asset["name"].as<String>();
    if (name.endsWith(".bin")) {
      binUrl = asset["browser_download_url"].as<String>();
      Serial.print("OTA: Found binary: ");
      Serial.println(name);
      break;
    }
  }
  
  if (binUrl.length() == 0) {
    Serial.println("OTA: No .bin file in release assets");
    sendStatusToMega("CMD:OTA_FAIL");
    return;
  }
  
  // Step 5: Download and flash
  Serial.print("OTA: Downloading from: ");
  Serial.println(binUrl);
  sendStatusToMega("CMD:OTA_START");
  
  // Disable watchdog during OTA (download can take >30s)
  esp_task_wdt_delete(NULL);
  Serial.println("OTA: Watchdog disabled for download");
  
  WiFiClientSecure otaClient;
  otaClient.setInsecure(); // Skip cert for GitHub CDN
  
  // Set update callbacks for progress
  httpUpdate.onStart([]() {
    Serial.println("OTA: Download started...");
  });
  httpUpdate.onEnd([]() {
    Serial.println("OTA: Download complete!");
  });
  httpUpdate.onProgress([](int current, int total) {
    Serial.printf("OTA: Progress %d/%d bytes (%d%%)\n", current, total, (current * 100) / total);
  });
  httpUpdate.onError([](int err) {
    Serial.printf("OTA: Error[%d]: %s\n", err, httpUpdate.getLastErrorString().c_str());
  });
  
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(otaClient, binUrl);
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("OTA: FAILED (Error %d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      sendStatusToMega("CMD:OTA_FAIL");
      // Re-enable watchdog
      esp_task_wdt_add(NULL);
      break;
      
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("OTA: No update needed");
      sendStatusToMega("CMD:OTA_UPTODATE");
      // Re-enable watchdog
      esp_task_wdt_add(NULL);
      break;
      
    case HTTP_UPDATE_OK:
      Serial.println("OTA: SUCCESS! Rebooting...");
      sendStatusToMega("CMD:OTA_DONE");
      delay(1000);
      ESP.restart(); // Reboot with new firmware
      break;
  }
}
