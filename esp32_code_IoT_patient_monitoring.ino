#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// Sensor libraries
#include <MAX30105.h>
#include <heartRate.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Adafruit_MLX90614.h>
#include <MPU6050.h>

#include "config.h"

// ========== SENSOR OBJECTS ==========
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU6050 mpu;
SparkFun_Bio_Sensor_Hub bioHub(12, 13); 

// ========== NETWORK OBJECTS ==========
WiFiClient espClient;
PubSubClient client(espClient);

// ========== GLOBAL VARIABLES ==========
// Heart rate calculation
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Sensor data structure
struct SensorData {
  int heartRate;
  int spo2;
  float temperature;
  int ecgValue;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  bool fallDetected;
  int batteryLevel;
};

// Timing and state variables
unsigned long lastSendTime = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000;
bool isBatteryPower = false;

// ========== FORWARD DECLARATIONS ==========
void printSensorReadings(SensorData data);
bool initializeSensors();
void checkPowerSource();
SensorData readAllSensors();
bool checkCriticalAlerts(SensorData data);
void checkAndSendAlerts(SensorData data);
void sendData(SensorData data, bool isCritical);
void sendAlert(SensorData data);
void setupNetwork();
bool ensureNetworkConnection();
void setupWiFi();
bool reconnectMQTT();
int readBatteryLevel();
void storeDataOffline(SensorData data);
void sendOfflineData();
void goToDeepSleep();
void setPinsToLowPower();
void batteryPowerLoop();
void mainsPowerLoop();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println("\n=== REMOTE PATIENT MONITORING SYSTEM ===");
  Serial.println("Initializing...");
  
  // Initialize EEPROM for offline data storage
  EEPROM.begin(512);
  
  // Check power source
  checkPowerSource();
  
  // Initialize sensors
  if (!initializeSensors()) {
    Serial.println("CRITICAL: Sensor initialization failed!");
    goToDeepSleep();
  }
  
  // Connect to network (if not in deep sleep mode)
  if (!isBatteryPower) {
    setupNetwork();
  }
  
  Serial.println("=== SYSTEM INITIALIZATION COMPLETE ===");
  Serial.printf("Device ID: %s\n", PATIENT_ID);
  Serial.printf("Send Interval: %d ms\n", SEND_INTERVAL);
  Serial.printf("Power Mode: %s\n", isBatteryPower ? "BATTERY" : "MAINS");
  Serial.println("======================================\n");
}

// ========== MAIN LOOP ==========
void loop() {
  if (isBatteryPower) {
    batteryPowerLoop();
  } else {
    mainsPowerLoop();
  }
}

// ========== BATTERY POWER MODE ==========
void batteryPowerLoop() {
  // Read all sensors
  SensorData data = readAllSensors();
  
  // Print readings for monitoring
  printSensorReadings(data);
  
  // Check for critical alerts (send immediately)
  if (checkCriticalAlerts(data)) {
    Serial.println("CRITICAL ALERT DETECTED!");
    if (ensureNetworkConnection()) {
      sendData(data, true); // Send as critical alert
    } else {
      storeDataOffline(data); // Store for later transmission
      Serial.println("Network unavailable - stored offline");
    }
  }
  
  // Regular data transmission on battery
  if (millis() - lastSendTime > SEND_INTERVAL) {
    if (ensureNetworkConnection()) {
      sendData(data, false);
      sendOfflineData(); // Send any stored offline data
    } else {
      storeDataOffline(data);
    }
    lastSendTime = millis();
  }
  
  // Enter deep sleep to save power
  Serial.printf("Entering deep sleep for %d seconds...\n", DEEP_SLEEP_TIME);
  setPinsToLowPower();
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * 1000000);
  esp_deep_sleep_start();
}

// ========== MAINS POWER MODE ==========
void mainsPowerLoop() {
  // Maintain network connection
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  // Regular data transmission
  if (millis() - lastSendTime > SEND_INTERVAL) {
    SensorData data = readAllSensors();
    
    // Print readings for monitoring
    printSensorReadings(data);
    
    sendData(data, false);
    
    // Check for alerts
    checkAndSendAlerts(data);
    
    lastSendTime = millis();
  }
  
  // Small delay to prevent overwhelming the system
  delay(100);
}

// ========== SENSOR INITIALIZATION ==========
bool initializeSensors() {
  Serial.println("Initializing sensors...");
  
  // Initialize I2C
  Wire.begin(21, 22);
  delay(100);
  
  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FAILED: MAX30102 not found!");
    return false;
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("SUCCESS: MAX30102 initialized");
  
  // Initialize Bio Sensor Hub
  if (bioHub.begin() != 0) {
    Serial.println("FAILED: Bio Sensor Hub not found!");
    return false;
  }
  bioHub.configBpm(MODE_ONE);
  Serial.println("SUCCESS: Bio Sensor Hub initialized");
  
  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("FAILED: MLX90614 not found!");
    return false;
  }
  Serial.println("SUCCESS: MLX90614 initialized");
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("FAILED: MPU6050 not found!");
    return false;
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println("SUCCESS: MPU6050 initialized");
  
  // Configure ECG pins
  pinMode(34, INPUT); // ECG OUTPUT
  pinMode(36, INPUT); // LO+
  pinMode(39, INPUT); // LO-
  Serial.println("SUCCESS: AD8232 ECG pins configured");
  
  Serial.println("ALL SENSORS INITIALIZED SUCCESSFULLY");
  return true;
}

// ========== SENSOR READING ==========
SensorData readAllSensors() {
  SensorData data;
  
  // Read MAX30102 for heart rate using IR
  long irValue = particleSensor.getIR();
  
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      
      // Calculate average heart rate
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  data.heartRate = beatAvg;
  
  // Read Bio Sensor Hub for SpO2 and alternative BPM
  bioData body; 
  body = bioHub.readBpm();
  data.spo2 = body.oxygen;
  // Use Bio Hub heart rate if MAX30102 reading is invalid
  if (data.heartRate == 0) {
    data.heartRate = body.heartRate;
  }
  
  // Read MLX90614 temperature
  data.temperature = mlx.readObjectTempC();
  
  // Read AD8232 ECG
  data.ecgValue = analogRead(34);
  
  // Read MPU6050 accelerometer and gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful values
  data.accelX = ax / 16384.0;
  data.accelY = ay / 16384.0;
  data.accelZ = az / 16384.0;
  data.gyroX = gx / 131.0;
  data.gyroY = gy / 131.0;
  data.gyroZ = gz / 131.0;
  
  // Fall detection
  float totalAccel = sqrt(data.accelX * data.accelX + 
                         data.accelY * data.accelY + 
                         data.accelZ * data.accelZ);
  data.fallDetected = (totalAccel > 3.0);
  
  // Battery level
  data.batteryLevel = readBatteryLevel();
  
  return data;
}

// ========== PRINT SENSOR READINGS ==========
void printSensorReadings(SensorData data) {
  Serial.println("\n=== SENSOR READINGS ===");
  Serial.println("--- VITAL SIGNS ---");
  Serial.printf("Heart Rate: %d bpm\n", data.heartRate);
  Serial.printf("SpO2: %d%%\n", data.spo2);
  Serial.printf("Body Temperature: %.2f °C\n", data.temperature);
  Serial.printf("ECG Raw Value: %d\n", data.ecgValue);
  
  Serial.println("--- ACTIVITY DATA ---");
  Serial.printf("Acceleration: X=%.2fg, Y=%.2fg, Z=%.2fg\n", 
                data.accelX, data.accelY, data.accelZ);
  Serial.printf("Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s\n", 
                data.gyroX, data.gyroY, data.gyroZ);
  Serial.printf("Fall Detected: %s\n", data.fallDetected ? "YES" : "NO");
  
  Serial.println("--- SYSTEM STATUS ---");
  Serial.printf("Battery Level: %d%%\n", data.batteryLevel);
  Serial.printf("WiFi Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.printf("MQTT Status: %s\n", client.connected() ? "Connected" : "Disconnected");
  Serial.printf("Power Mode: %s\n", isBatteryPower ? "Battery" : "Mains");
  Serial.println("=====================\n");
}

// ========== ALERT HANDLING ==========
bool checkCriticalAlerts(SensorData data) {
  return (data.heartRate > MAX_HEART_RATE || 
          data.heartRate < MIN_HEART_RATE ||
          data.spo2 < MIN_SPO2 ||
          data.temperature > MAX_TEMPERATURE ||
          data.fallDetected);
}

void checkAndSendAlerts(SensorData data) {
  if (checkCriticalAlerts(data)) {
    sendAlert(data);
  }
}

void sendAlert(SensorData data) {
  StaticJsonDocument<256> doc;
  doc["patient_id"] = PATIENT_ID;
  doc["timestamp"] = millis();
  doc["alert"] = "CRITICAL";
  
  if (data.heartRate > MAX_HEART_RATE) doc["condition"] = "HIGH_HEART_RATE";
  else if (data.heartRate < MIN_HEART_RATE) doc["condition"] = "LOW_HEART_RATE";
  else if (data.spo2 < MIN_SPO2) doc["condition"] = "LOW_SPO2";
  else if (data.temperature > MAX_TEMPERATURE) doc["condition"] = "HIGH_TEMPERATURE";
  else if (data.fallDetected) doc["condition"] = "FALL_DETECTED";
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.connected()) {
    client.publish("patient/alerts", payload.c_str());
    Serial.println("ALERT SENT: " + payload);
  }
}

// ========== DATA TRANSMISSION ==========
void sendData(SensorData data, bool isCritical) {
  StaticJsonDocument<512> doc;
  
  doc["patient_id"] = PATIENT_ID;
  doc["timestamp"] = millis();
  doc["critical"] = isCritical;
  
  JsonObject vitals = doc.createNestedObject("vitals");
  vitals["heart_rate"] = data.heartRate;
  vitals["spo2"] = data.spo2;
  vitals["temperature"] = data.temperature;
  vitals["ecg_raw"] = data.ecgValue;
  vitals["battery"] = data.batteryLevel;
  
  JsonObject activity = doc.createNestedObject("activity");
  activity["accel_x"] = data.accelX;
  activity["accel_y"] = data.accelY;
  activity["accel_z"] = data.accelZ;
  activity["fall_detected"] = data.fallDetected;
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.connected()) {
    client.publish(MQTT_TOPIC, payload.c_str());
    Serial.println("Data sent to MQTT: " + payload);
  } else {
    Serial.println("MQTT not connected - data not sent");
  }
}

// ========== NETWORK FUNCTIONS ==========
void setupNetwork() {
  setupWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  reconnectMQTT();
}

bool ensureNetworkConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }
  if (!client.connected()) {
    return reconnectMQTT();
  }
  return true;
}

void setupWiFi() {
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi CONNECTED");
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi FAILED");
  }
}

bool reconnectMQTT() {
  Serial.println("Connecting to MQTT...");
  String clientId = "ESP32PatientMonitor-" + String(random(0xffff), HEX);
  
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("MQTT CONNECTED");
    return true;
  } else {
    Serial.printf("MQTT FAILED, rc=%d\n", client.state());
    return false;
  }
}

// ========== UTILITY FUNCTIONS ==========
void checkPowerSource() {
  // Simplified check - in real implementation, use proper power monitoring
  isBatteryPower = false; // Assume mains power for prototype
}

int readBatteryLevel() {
  // Simplified - in real implementation, read from ADC with voltage divider
  return 85; // Simulated 85% battery
}

void storeDataOffline(SensorData data) {
  // Store data in EEPROM for later transmission
  Serial.println("STORING DATA OFFLINE (network unavailable)");
  // Implementation would write to EEPROM here
}

void sendOfflineData() {
  // Send any stored offline data when network is available
  Serial.println("SENDING PREVIOUSLY STORED OFFLINE DATA");
  // Implementation would read from EEPROM and send here
}

void goToDeepSleep() {
  Serial.println("CRITICAL ERROR - Entering deep sleep");
  esp_deep_sleep_start();
}

void setPinsToLowPower() {
  // Set all unused pins to low power state
  int pins[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
  for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
    pinMode(pins[i], INPUT_PULLDOWN);
  }
  Serial.println("All pins set to low power state");
}
