// config.h - Configuration and secrets
#define WIFI_SSID "Your_WiFi_SSID"
#define WIFI_PASSWORD "Your_WiFi_Password"
#define MQTT_SERVER "your.mqtt.broker.ip"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user"
#define MQTT_PASSWORD "mqtt_password"
#define MQTT_TOPIC "patient/vitals"

// Device configuration
#define PATIENT_ID "patient_001"
#define SEND_INTERVAL 30000  // 30 seconds
#define DEEP_SLEEP_TIME 300  // 5 minutes sleep when on battery

// Alert thresholds
#define MAX_HEART_RATE 120
#define MIN_HEART_RATE 40
#define MAX_TEMPERATURE 38.5  // °C
#define MIN_SPO2 90           // %

// Debug settings
#define DEBUG_LEVEL 2  // 0=Off, 1=Minimal, 2=Detailed
