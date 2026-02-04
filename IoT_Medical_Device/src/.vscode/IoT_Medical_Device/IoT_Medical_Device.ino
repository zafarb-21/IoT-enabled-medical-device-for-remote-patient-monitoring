#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WiFiClientSecure.h>

// Sensor libraries
#include <MAX30105.h>
#include <heartRate.h>
#include "Adafruit_MLX90614.h"
#include "MPU6050.h"

// ========== CONFIGURATION ==========
const char* WIFI_SSID = "Taffy";
const char* WIFI_PASSWORD = "TaffyBillions";

// HiveMQ Configuration
const char* MQTT_SERVER = "a5ef99d0bdcd45feb91d4bd3881df6de.s1.eu.hivemq.cloud";
const int MQTT_PORT = 8883;
const char* MQTT_USER = "admin";
const char* MQTT_PASSWORD = "Trishbasvi@2021";
const char* MQTT_TOPIC = "patient/vitals";
const char* PATIENT_ID = "patient_001";

// AD8232 ECG Configuration
#define ECG_PIN 34
#define LO_PLUS_PIN 35
#define LO_MINUS_PIN 32
#define SDN_PIN 25

// 🔔 ALERT SYSTEM CONFIGURATION - ADDED
#define LED_GREEN 5    // GPIO5 - Green LED (Normal)
#define LED_ORANGE 18   // GPIO18 - Orange LED (Warning)
#define LED_RED 19      // GPIO19 - Red LED (Critical)
#define BUZZER_PIN 23   // GPIO23 - Buzzer

// Alert thresholds - ENHANCED WITH WARNING LEVELS
const int MAX_HEART_RATE = 120;
const int MIN_HEART_RATE = 50;
const int WARNING_MAX_HR = 110;    // Warning threshold
const int WARNING_MIN_HR = 55;     // Warning threshold

const int MIN_SPO2 = 90;
const int WARNING_MIN_SPO2 = 92;   // Warning threshold

const float MAX_TEMPERATURE = 38.0;
const float WARNING_MAX_TEMP = 37.9; // Warning threshold

// Alert states - ADDED
enum AlertState {
  ALERT_NORMAL,      // Green LED
  ALERT_WARNING,     // Orange LED
  ALERT_CRITICAL     // Red LED + Buzzer
};

AlertState currentAlert = ALERT_NORMAL;
unsigned long lastAlertChange = 0;
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;

// ECG parameters
#define ECG_BUFFER_SIZE 250
#define HEART_RATE_SAMPLES 10
const unsigned long ECG_SEND_INTERVAL = 1000;
const unsigned long SEND_INTERVAL = 5000;

// ========== SENSOR OBJECTS ==========
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU6050 mpu;

// ========== NETWORK OBJECTS ==========
WiFiClientSecure espClient;
PubSubClient client(espClient);

// ========== GLOBAL VARIABLES ==========
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t bufferLength = 0;
int32_t spo2 = 0;
int8_t validSPO2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;

long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// AD8232 ECG variables
int ecgBuffer[ECG_BUFFER_SIZE];
int ecgIndex = 0;
unsigned long lastEcgSend = 0;
bool leadOffDetected = false;
bool ecgQualityGood = true;
int heartRateSamples[HEART_RATE_SAMPLES];
int hrSampleIndex = 0;
unsigned long lastPeakTime = 0;
float ecgHeartRate = 72.0;

struct SensorData {
  int heartRate = 0;
  int spo2 = 0;
  float temperature = 0.0;
  int ecgValue = 0;
  float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
  bool fallDetected = false;
  int batteryLevel = 0;
  bool leadOff = false;
  bool ecgSignalQuality = true;
  float ecgBasedHeartRate = 0.0;
};

unsigned long lastSendTime = 0;
bool isBatteryPower = false;

// 🔔 ALERT SYSTEM FUNCTIONS - MOVED TO TOP (BEFORE setup())
void initializeAlertSystem() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Turn off all LEDs initially
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("Alert system initialized");
  Serial.println("LED Pins: Green=GPIO5, Orange=GPIO18, Red=GPIO19, Buzzer=GPIO23");
}

void setAlert(AlertState newAlert, String reason = "") {  // Changed to String
  if (currentAlert != newAlert) {
    currentAlert = newAlert;
    lastAlertChange = millis();
    
    // Turn off all LEDs first
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_RED, LOW);
    
    // Activate appropriate LED
    switch (newAlert) {
      case ALERT_NORMAL:
        digitalWrite(LED_GREEN, HIGH);
        Serial.println("✅ Alert: NORMAL - " + reason);
        break;
      case ALERT_WARNING:
        digitalWrite(LED_ORANGE, HIGH);
        Serial.println("⚠️ Alert: WARNING - " + reason);
        break;
      case ALERT_CRITICAL:
        digitalWrite(LED_RED, HIGH);
        buzzerActive = true;
        buzzerStartTime = millis();
        Serial.println("🚨 Alert: CRITICAL - " + reason);
        break;
    }
  }
}

void checkAndUpdateAlerts(SensorData data) {
  // CRITICAL CONDITIONS (Red LED + Buzzer)
  if (data.fallDetected) {
    setAlert(ALERT_CRITICAL, "FALL DETECTED!");
    return;
  }
  
  if (data.heartRate >= MAX_HEART_RATE || data.heartRate <= MIN_HEART_RATE) {
    setAlert(ALERT_CRITICAL, "Heart Rate CRITICAL: " + String(data.heartRate) + " bpm");
    return;
  }
  
  if (data.spo2 < MIN_SPO2) {
    setAlert(ALERT_CRITICAL, "SpO2 CRITICAL: " + String(data.spo2) + "%");
    return;
  }
  
  if (data.temperature >= MAX_TEMPERATURE) {
    setAlert(ALERT_CRITICAL, "Temperature CRITICAL: " + String(data.temperature) + "°C");
    return;
  }
  
  if (data.leadOff) {
    setAlert(ALERT_WARNING, "Electrodes DISCONNECTED!");
    return;
  }
  
  // WARNING CONDITIONS (Orange LED)
  if (data.heartRate >= WARNING_MAX_HR || data.heartRate <= WARNING_MIN_HR) {
    setAlert(ALERT_WARNING, "Heart Rate warning: " + String(data.heartRate) + " bpm");
    return;
  }
  
  if (data.spo2 < WARNING_MIN_SPO2) {
    setAlert(ALERT_WARNING, "SpO2 warning: " + String(data.spo2) + "%");
    return;
  }
  
  if (data.temperature >= WARNING_MAX_TEMP) {
    setAlert(ALERT_WARNING, "Temperature warning: " + String(data.temperature) + "°C");
    return;
  }
  
  if (!data.ecgSignalQuality) {
    setAlert(ALERT_WARNING, "Poor ECG signal quality");
    return;
  }
  
  // NORMAL CONDITIONS (Green LED)
  setAlert(ALERT_NORMAL, "All vitals normal");
}

void manageBuzzer() {
  if (!buzzerActive) return;
  
  // Pulsing pattern for critical alerts: 500ms ON, 500ms OFF
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - buzzerStartTime;
  
  if (elapsed % 1000 < 500) {
    digitalWrite(BUZZER_PIN, HIGH);  // Buzzer ON
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Buzzer OFF
  }
  
  // Stop buzzer if alert is no longer critical
  if (currentAlert != ALERT_CRITICAL) {
    buzzerActive = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ========== ECG FUNCTIONS ==========
void initializeECGSensor() {
  Serial.println("Initializing AD8232 ECG Sensor...");
  
  // Configure pins
  pinMode(ECG_PIN, INPUT);
  pinMode(LO_PLUS_PIN, INPUT);  //
  pinMode(LO_MINUS_PIN, INPUT); // 
  pinMode(SDN_PIN, OUTPUT);
  
  // Start with AD8232 ON (SDN = LOW)
  digitalWrite(SDN_PIN, LOW);
  
  Serial.printf("ECG Pin: GPIO%d\n", ECG_PIN);
  Serial.printf("LO+ Pin: GPIO%d\n", LO_PLUS_PIN);
  Serial.printf("LO- Pin: GPIO%d\n", LO_MINUS_PIN);
  Serial.printf("SDN Pin: GPIO%d\n", SDN_PIN);
  Serial.println("AD8232 initialized and powered ON");
}

void checkElectrodeConnection() {
  bool loPlus = digitalRead(LO_PLUS_PIN);
  bool loMinus = digitalRead(LO_MINUS_PIN);
  
  Serial.println("\n=== ELECTRODE CONNECTION CHECK ===");
  Serial.printf("LO+ Status: %s\n", loPlus ? "HIGH (Electrode OK)" : "LOW (Electrode OFF)");
  Serial.printf("LO- Status: %s\n", loMinus ? "HIGH (Electrode OK)" : "LOW (Electrode OFF)");
  
  if (!loPlus && !loMinus) {
    Serial.println("⚠️ ALERT: BOTH ELECTRODES DISCONNECTED!");
  } else if (!loPlus) {
    Serial.println("⚠️ ALERT: RIGHT ARM ELECTRODE DISCONNECTED!");
  } else if (!loMinus) {
    Serial.println("⚠️ ALERT: LEFT ARM ELECTRODE DISCONNECTED!");
  } else {
    Serial.println("✅ Both electrodes properly connected");
  }
  Serial.println("=====================================\n");
}

void readECGData() {
  // Check lead-off status
  bool loPlus = digitalRead(LO_PLUS_PIN);
  bool loMinus = digitalRead(LO_MINUS_PIN);
  leadOffDetected = (!loPlus || !loMinus);
  
  // Read ECG value
  int ecgValue = analogRead(ECG_PIN);
  
  // Store in buffer
  ecgBuffer[ecgIndex] = ecgValue;
  ecgIndex = (ecgIndex + 1) % ECG_BUFFER_SIZE;
  
  // Perform QRS detection for heart rate calculation (if electrodes are connected)
  if (!leadOffDetected) {
    detectQRS(ecgValue);
  }
  
  // Check signal quality
  checkECGQuality(ecgValue);
}

void detectQRS(int ecgValue) {
  static int lastValue = 0;
  static int threshold = 2048; // Adjust based on your signal baseline
  static bool aboveThreshold = false;
  static unsigned long lastPeak = 0;
  static int peakCount = 0;
  
  // Simple threshold-based QRS detection
  if (ecgValue > threshold && !aboveThreshold) {
    aboveThreshold = true;
    
    // Valid peak detection (debouncing)
    if (millis() - lastPeak > 300) { // Minimum 300ms between beats (200 BPM max)
      unsigned long currentTime = millis();
      
      if (lastPeak > 0) {
        long interval = currentTime - lastPeak;
        
        // Valid heart rate range: 30-200 BPM
        if (interval > 300 && interval < 2000) {
          float instantBPM = 60000.0 / interval;
          
          // Filter unrealistic values
          if (instantBPM >= 30 && instantBPM <= 200) {
            // Store in buffer for averaging
            heartRateSamples[hrSampleIndex] = (int)instantBPM;
            hrSampleIndex = (hrSampleIndex + 1) % HEART_RATE_SAMPLES;
            
            // Calculate average
            int sum = 0;
            int count = 0;
            for (int i = 0; i < HEART_RATE_SAMPLES; i++) {
              if (heartRateSamples[i] > 0) {
                sum += heartRateSamples[i];
                count++;
              }
            }
            
            if (count > 0) {
              ecgHeartRate = sum / (float)count;
            }
          }
        }
      }
      lastPeak = currentTime;
      peakCount++;
    }
    
  } else if (ecgValue < threshold - 100) {
    aboveThreshold = false;
  }
  
  lastValue = ecgValue;
}

void checkECGQuality(int ecgValue) {
  static int lastValues[5] = {0};
  static int valueIndex = 0;
  
  // Store recent values
  lastValues[valueIndex] = ecgValue;
  valueIndex = (valueIndex + 1) % 5;
  
  // Check for flat line (poor contact or no signal)
  int range = 0;
  int minVal = 4096, maxVal = 0;
  
  for (int i = 0; i < 5; i++) {
    if (lastValues[i] < minVal) minVal = lastValues[i];
    if (lastValues[i] > maxVal) maxVal = lastValues[i];
  }
  range = maxVal - minVal;
  
  // If signal range is too small, quality is poor
  ecgQualityGood = (range > 50) && !leadOffDetected;
}

void sendECGData() {
  if (!client.connected()) return;
  
  // Create JSON for ECG stream data
  StaticJsonDocument<1536> doc; // Larger for ECG array
  
  doc["patient_id"] = PATIENT_ID;
  doc["type"] = "ecg_stream";
  doc["timestamp"] = millis();
  doc["lead_off"] = leadOffDetected;
  doc["signal_quality"] = ecgQualityGood ? "good" : "poor";
  doc["ecg_heart_rate"] = ecgHeartRate;
  
  JsonArray ecg_data = doc.createNestedArray("ecg_samples");
  // Send last 100 samples (to keep message size reasonable)
  for (int i = 0; i < min(100, ECG_BUFFER_SIZE); i++) {
    int idx = (ecgIndex - i - 1 + ECG_BUFFER_SIZE) % ECG_BUFFER_SIZE;
    ecg_data.add(ecgBuffer[idx]);
  }
  
  String payload;
  serializeJson(doc, payload);
  
  // Publish to separate ECG topic
  client.publish("patient/ecg_stream", payload.c_str());
  
  // Debug: Print ECG status occasionally
  static unsigned long lastEcgStatus = 0;
  if (millis() - lastEcgStatus > 10000) {
    lastEcgStatus = millis();
    Serial.printf("ECG Status: HR=%.1f bpm, Lead-off=%s, Quality=%s\n",
                  ecgHeartRate,
                  leadOffDetected ? "YES" : "NO",
                  ecgQualityGood ? "GOOD" : "POOR");
  }
}

// ========== SDN (SHUTDOWN) CONTROL ==========
void setECGSensorPower(bool powerOn) {
  if (powerOn) {
    digitalWrite(SDN_PIN, LOW);  // Turn ON AD8232
    delay(50);                   // Wait for stabilization
    Serial.println("AD8232: Powered ON");
  } else {
    digitalWrite(SDN_PIN, HIGH); // Turn OFF AD8232
    Serial.println("AD8232: Powered OFF (Shutdown)");
  }
}

// ========== SENSOR INITIALIZATION ==========
bool initializeSensors() {
  Serial.println("Initializing sensors...");
  
  
  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found - continuing without it");
  } else {
    byte ledBrightness = 0x1F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 initialized");
  }
  
  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("MLX90614 not found - continuing without it");
  } else {
    Serial.println("MLX90614 initialized");
  }
  
  // Initialize MPU6050
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    mpu = MPU6050(0x68);
    mpu.initialize();
    if (mpu.testConnection()) {
      Serial.println("MPU6050 found at address 0x68");
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    }
  } else {
    Serial.println("MPU6050 not found");
  }
  
  Serial.println("Sensor initialization complete");
  return true;
}

// ========== SENSOR READING FUNCTIONS ==========
void calculateSpo2AndHeartRate() {
  if (bufferLength < BUFFER_SIZE) {
    redBuffer[bufferLength] = particleSensor.getRed();
    irBuffer[bufferLength] = particleSensor.getIR();
    bufferLength++;
  }

  if (bufferLength == BUFFER_SIZE) {
    // Use simulated data for now
    spo2 = 95 + random(-2, 3);
    heartRate = 72 + random(-5, 6);
    validSPO2 = 1;
    validHeartRate = 1;
    bufferLength = 0;
  }
}

SensorData readAllSensors() {
  SensorData data;
  
  // Calculate SpO2 and heart rate from MAX30102
  calculateSpo2AndHeartRate();
  
  // Use calculated values or fallbacks
  data.spo2 = (validSPO2 == 1) ? spo2 : 95;
  
  // Priority: Use ECG heart rate if available and good quality
  // Otherwise use MAX30102 heart rate
  if (ecgQualityGood && !leadOffDetected && ecgHeartRate > 30 && ecgHeartRate < 200) {
    data.heartRate = (int)ecgHeartRate;
    data.ecgBasedHeartRate = ecgHeartRate;
  } else {
    data.heartRate = (validHeartRate == 1) ? heartRate : 72;
    data.ecgBasedHeartRate = 0.0; // Not valid
  }
  
  // Read temperature
  data.temperature = mlx.readObjectTempC();
  if (isnan(data.temperature) || data.temperature == 0) {
    data.temperature = 36.5 + (random(0, 100) / 100.0);
  }
  
  // Read current ECG value
  data.ecgValue = analogRead(ECG_PIN);
  data.leadOff = leadOffDetected;
  data.ecgSignalQuality = ecgQualityGood;
  
  // Read accelerometer/gyro if available
  if (mpu.testConnection()) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    data.accelX = ax / 16384.0;
    data.accelY = ay / 16384.0;
    data.accelZ = az / 16384.0;
    data.gyroX = gx / 131.0;
    data.gyroY = gy / 131.0;
    data.gyroZ = gz / 131.0;
    
    float totalAccel = sqrt(data.accelX * data.accelX + 
                           data.accelY * data.accelY + 
                           data.accelZ * data.accelZ);
    data.fallDetected = (totalAccel > 3.0);
  }
  
  // Battery level
  data.batteryLevel = readBatteryLevel();
  
  return data;
}

// ========== UTILITY FUNCTIONS ==========
void checkPowerSource() {
  isBatteryPower = false; // Assume mains power
}

int readBatteryLevel() {
  return 85; // Simulated battery level
}

// 🎚️ ENHANCED PRINT FUNCTION WITH ALERT STATUS - MODIFIED
void printSensorReadings(SensorData data) {
  Serial.println("\n=== SENSOR READINGS ===");
  Serial.println("--- VITAL SIGNS ---");
  Serial.printf("Heart Rate: %d bpm (ECG: %.1f bpm)\n", data.heartRate, data.ecgBasedHeartRate);
  Serial.printf("SpO2: %d%%\n", data.spo2);
  Serial.printf("Temperature: %.2f °C\n", data.temperature);
  Serial.printf("ECG Raw: %d\n", data.ecgValue);
  
  Serial.println("--- ECG STATUS ---");
  Serial.printf("Lead-off Detected: %s\n", data.leadOff ? "YES ⚠️" : "NO ✅");
  Serial.printf("ECG Signal Quality: %s\n", data.ecgSignalQuality ? "GOOD ✅" : "POOR ⚠️");
  
  Serial.println("--- ACTIVITY ---");
  Serial.printf("Acceleration: X=%.2fg, Y=%.2fg, Z=%.2fg\n", 
                data.accelX, data.accelY, data.accelZ);
  Serial.printf("Fall Detected: %s\n", data.fallDetected ? "YES ⚠️" : "NO ✅");
  
  // 🔔 ADDED ALERT STATUS DISPLAY
  Serial.println("--- ALERT SYSTEM ---");
  switch (currentAlert) {
    case ALERT_NORMAL:
      Serial.println("Status: 🟢 NORMAL (Green LED ON)");
      break;
    case ALERT_WARNING:
      Serial.println("Status: 🟠 WARNING (Orange LED ON)");
      break;
    case ALERT_CRITICAL:
      Serial.println("Status: 🔴 CRITICAL (Red LED ON + Buzzer)");
      break;
  }
  
  Serial.println("--- SYSTEM ---");
  Serial.printf("Battery: %d%%\n", data.batteryLevel);
  Serial.printf("WiFi: %s | MQTT: %s\n", 
                WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                client.connected() ? "Connected" : "Disconnected");
  Serial.println("=====================\n");
}

// 📤 ENHANCED DATA TRANSMISSION WITH ALERT STATUS - MODIFIED
void sendData(SensorData data, bool isCritical) {
  StaticJsonDocument<768> doc;
  
  doc["patient_id"] = PATIENT_ID;
  doc["timestamp"] = millis();
  doc["critical"] = isCritical;
  
  // 🔔 ADD ALERT LEVEL TO JSON
  doc["alert_level"] = (currentAlert == ALERT_NORMAL) ? "normal" : 
                      (currentAlert == ALERT_WARNING) ? "warning" : "critical";
  
  JsonObject vitals = doc["vitals"].to<JsonObject>();
  vitals["heart_rate"] = data.heartRate;
  vitals["spo2"] = data.spo2;
  vitals["temperature"] = data.temperature;
  vitals["ecg_raw"] = data.ecgValue;
  vitals["ecg_heart_rate"] = data.ecgBasedHeartRate;
  vitals["lead_off"] = data.leadOff;
  vitals["ecg_quality"] = data.ecgSignalQuality ? "good" : "poor";
  vitals["battery"] = data.batteryLevel;
  
  JsonObject activity = doc["activity"].to<JsonObject>();
  activity["accel_x"] = data.accelX;
  activity["accel_y"] = data.accelY;
  activity["accel_z"] = data.accelZ;
  activity["gyro_x"] = data.gyroX;
  activity["gyro_y"] = data.gyroY;
  activity["gyro_z"] = data.gyroZ;
  activity["fall_detected"] = data.fallDetected;
  
  JsonObject system = doc["system"].to<JsonObject>();
  system["rssi"] = WiFi.RSSI();
  system["free_heap"] = ESP.getFreeHeap();
  system["uptime"] = millis();
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.connected()) {
    if (client.publish(MQTT_TOPIC, payload.c_str())) {
      Serial.println("✅ Data sent to HiveMQ");
      Serial.println("Payload: " + payload);
    } else {
      Serial.println("❌ Failed to publish to HiveMQ");
    }
  } else {
    Serial.println("❌ MQTT not connected - data not sent");
  }
}

// ========== NETWORK FUNCTIONS ==========
void setupNetwork() {
  setupWiFi();

  espClient.setInsecure();   // 🔑 REQUIRED for HiveMQ Cloud (TLS)
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setBufferSize(2048);
  reconnectMQTT();
}

void setupWiFi() {
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting");
  unsigned long startTime = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Timeout after 30 seconds
    if (millis() - startTime > 30000) {
      Serial.println("\n❌ WiFi connection failed!");
      Serial.println("Please check:");
      Serial.println("1. Network is 2.4GHz (not 5GHz)");
      Serial.println("2. ESP32 is in range");
      Serial.println("3. Router is not blocking device");
      Serial.println("4. Wrong Password");
      return;
    }
  }
  
  Serial.println("\n✅ WiFi CONNECTED!");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
}

bool reconnectMQTT() {
  // Ensure WiFi is connected first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - reconnecting...");
    setupWiFi();
    
    if (WiFi.status() != WL_CONNECTED) {
      return false;
    }
  }
  
  Serial.println("Connecting to HiveMQ...");
  
  // Generate unique client ID
  String clientId = "ESP32Patient-" + String(random(0xffff), HEX);
  
  // Connect to HiveMQ with authentication
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("✅ CONNECTED to HiveMQ!");
    Serial.printf("Broker: %s:%d\n", MQTT_SERVER, MQTT_PORT);
    Serial.printf("Topic: %s\n", MQTT_TOPIC);
    Serial.println("Also publishing ECG stream to: patient/ecg_stream");
    return true;
  } else {
    Serial.printf("❌ MQTT connection failed, rc=%d\n", client.state());
    return false;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
   // Initialize I2C
   Wire.begin(21, 22);
  Wire.setClock(100000);   // 100 kHz for stability
  delay(100);
  
  Serial.println("\n=== REMOTE PATIENT MONITORING SYSTEM ===");
  Serial.println("Initializing...");
  
  // 🔔 INITIALIZE ALERT SYSTEM - ADDED
  initializeAlertSystem();
  
  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Initialize AD8232 pins
  initializeECGSensor();
  
  // Check power source
  checkPowerSource();
  
  // Initialize sensors
  if (!initializeSensors()) {
    Serial.println("CRITICAL: Sensor initialization failed!");
    // Show red alert for sensor failure
    setAlert(ALERT_CRITICAL, "Sensor initialization failed");
  }
  
  // Connect to network
  setupNetwork();
  
  Serial.println("=== SYSTEM INITIALIZATION COMPLETE ===");
  Serial.printf("Device ID: %s\n", PATIENT_ID);
  Serial.printf("Send Interval: %d ms\n", SEND_INTERVAL);
  Serial.println("AD8232 ECG Ready - Checking electrode connections...");
  checkElectrodeConnection();
  Serial.println("======================================\n");
  
  lastSendTime = millis();
  lastEcgSend = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  // Continuously read and process ECG data
  readECGData();
  
  // Send ECG data stream
  if (millis() - lastEcgSend > ECG_SEND_INTERVAL) {
    sendECGData();
    lastEcgSend = millis();
  }
  
  // Regular vital signs transmission
  if (millis() - lastSendTime > SEND_INTERVAL) {
    SensorData data = readAllSensors();
    
    // 🔔 CHECK ALERTS AND UPDATE LEDS - ADDED
    checkAndUpdateAlerts(data);
    
    // Print readings for monitoring
    printSensorReadings(data);
    
    // Send to HiveMQ
    sendData(data, (currentAlert == ALERT_CRITICAL));
    
    lastSendTime = millis();
  }
  
  // 🔔 MANAGE BUZZER (PULSING FOR CRITICAL ALERTS) - ADDED
  manageBuzzer();
  
  delay(10);
}