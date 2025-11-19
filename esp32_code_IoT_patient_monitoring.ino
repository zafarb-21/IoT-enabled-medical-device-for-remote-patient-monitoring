#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>

// Sensor Libraries
#include <MAX30105.h>
#include <heartRate.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Adafruit_MLX90614.h>
#include <AD8232.h>
#include <MPU6050.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "YOUR_MQTT_BROKER_IP";
const char* mqtt_topic = "patient/vitals";
const char* mqtt_username = "mqtt_user";
const char* mqtt_password = "mqtt_password";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensor objects
MAX30105 particleSensor;
AD8232 ecgSensor(36, 39, 34); // LO+, LO-, OUTPUT
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU6050 mpu;

// Bio Sensor Hub
#define RES_PIN 12
#define MFIO_PIN 13
SparkFun_Bio_Sensor_Hub bioHub(resp, mfio); 

// Variables for sensor data
bioData body; 
float temperature;
int ecgValue;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 5000; // Send data every 5 seconds

// Heart rate calculation
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize all sensors
  initializeSensors();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup MQTT
  client.setServer(mqtt_server, 1883);
  
  Serial.println("All sensors initialized. System ready.");
}

void initializeSensors() {
  Serial.println("Initializing sensors...");
  
  // Initialize MAX30102
  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 found!");
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
  } else {
    Serial.println("MAX30102 not found. Please check wiring/power.");
  }
  
  // Initialize Bio Sensor Hub (MAX32664)
  int result = bioHub.begin();
  if (result == 0) {
    Serial.println("Bio Sensor Hub started!");
    bioHub.configBpm(MODE_ONE); // Configure for just BPM and SPO2
  } else {
    Serial.println("Bio Sensor Hub failed to start.");
  }
  
  // Initialize MLX90614
  if (mlx.begin()) {
    Serial.println("MLX90614 found!");
  } else {
    Serial.println("MLX90614 not found. Please check wiring.");
  }
  
  // Initialize AD8232 ECG
  pinMode(34, INPUT); // Setup for ECG
  pinMode(36, INPUT);
  pinMode(39, INPUT);
  Serial.println("AD8232 ECG initialized");
  
  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  } else {
    Serial.println("MPU6050 connection failed");
  }
  
  delay(1000);
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void readAllSensors() {
  // Read MAX30102 for heart rate using IR
  long irValue = particleSensor.getIR();
  
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
  // Read Bio Sensor Hub for SpO2 and alternative BPM
  body = bioHub.readBpm();
  
  // Read MLX90614 temperature
  temperature = mlx.readObjectTempC();
  
  // Read AD8232 ECG
  ecgValue = analogRead(34);
  
  // Read MPU6050 accelerometer and gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful values
  accelX = ax / 16384.0; // For ±2g range
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;
  gyroX = gx / 131.0;    // For ±250°/sec range
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

String createJSONPayload() {
  StaticJsonDocument<512> doc;
  
  // Patient ID (you can modify this)
  doc["patient_id"] = "patient_001";
  
  // Vital signs
  JsonObject vitals = doc.createNestedObject("vitals");
  
  // Heart rate from MAX30102
  if (beatAvg > 0) {
    vitals["heart_rate_bpm"] = beatAvg;
  } else {
    vitals["heart_rate_bpm"] = body.heartRate;
  }
  
  // SpO2 from Bio Sensor Hub
  vitals["spo2"] = body.oxygen;
  
  // Temperature from MLX90614
  vitals["body_temp_c"] = temperature;
  
  // ECG value
  vitals["ecg_raw"] = ecgValue;
  
  // Activity data
  JsonObject activity = doc.createNestedObject("activity");
  activity["accel_x"] = accelX;
  activity["accel_y"] = accelY;
  activity["accel_z"] = accelZ;
  activity["gyro_x"] = gyroX;
  activity["gyro_y"] = gyroY;
  activity["gyro_z"] = gyroZ;
  
  // Calculate approximate fall detection
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  activity["total_acceleration"] = totalAccel;
  activity["fall_detected"] = (totalAccel > 3.0); // Threshold for fall detection
  
  // Timestamp
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void deepSleepIfNeeded() {
  // Check battery level or implement sleep logic here
  // For now, we'll keep it simple
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    // Stay awake
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Read all sensors
    readAllSensors();
    
    // Create JSON payload
    String payload = createJSONPayload();
    
    // Publish to MQTT
    if (client.connected()) {
      client.publish(mqtt_topic, payload.c_str());
      Serial.println("Data published:");
      Serial.println(payload);
    }
    
    // Print sensor readings to Serial for debugging
    printSensorReadings();
  }
  
  // Small delay to prevent overwhelming the system
  delay(100);
}

void printSensorReadings() {
  Serial.println("=== Sensor Readings ===");
  Serial.printf("Heart Rate: %d bpm\n", beatAvg > 0 ? beatAvg : body.heartRate);
  Serial.printf("SpO2: %d%%\n", body.oxygen);
  Serial.printf("Temperature: %.2f °C\n", temperature);
  Serial.printf("ECG Raw: %d\n", ecgValue);
  Serial.printf("Acceleration: X=%.2fg, Y=%.2fg, Z=%.2fg\n", accelX, accelY, accelZ);
  Serial.printf("Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s\n", gyroX, gyroY, gyroZ);
  Serial.println("=======================");
}