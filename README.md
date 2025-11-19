# IoT-Enabled Remote Patient Monitoring System

A graduate-level project demonstrating the integration of embedded systems and computer networks for affordable, scalable remote patient monitoring using ESP32 and IoT technologies.

## 📋 Project Overview

This system autonomously collects multiple physiological parameters from a patient, securely transmits the data over Wi-Fi using MQTT protocol, and displays real-time vitals on a cloud-based dashboard for remote clinical oversight.

### Key Features
- **Multi-Parameter Monitoring**: Heart rate, SpO2, body temperature, ECG, and activity/fall detection
- **Real-time Data Transmission**: Secure MQTT over Wi-Fi
- **Cloud Dashboard**: Real-time visualization with Grafana
- **Alert System**: Threshold-based notifications for abnormal readings
- **Low-Cost Design**: Total hardware cost under $50
- **Power Efficient**: Deep sleep capabilities for extended battery life

## 🛠 Hardware Components

| Component | Purpose | Cost (Approx.) |
|-----------|---------|----------------|
| ESP32 DevKit | Main microcontroller with Wi-Fi | $5 |
| MAX30102 | Heart rate & SpO2 monitoring | $10 |
| AD8232 | Single-lead ECG monitoring | $15 |
| MLX90614 | Non-contact body temperature | $10 |
| MPU6050 | Accelerometer & gyroscope for activity/fall detection | $2 |
| LiPo Battery | Portable power source | $5 |
| Breadboard & Wires | Prototyping | $3 |

**Total Estimated Cost: ~$50**

## 📦 Software Requirements

### Arduino IDE Libraries
```cpp
WiFi.h
PubSubClient.h
Wire.h
ArduinoJson.h
SparkFun MAX3010x Pulse and Proximity Sensor Library
SparkFun Bio Sensor Hub Library
Adafruit MLX90614 Library
MPU6050 by Electronic Cats
