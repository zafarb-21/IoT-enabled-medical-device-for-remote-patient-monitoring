# IoT-Enabled Remote Patient Monitoring System

A graduate-level project demonstrating the integration of embedded systems and computer networks for affordable, scalable remote patient monitoring using ESP32 and IoT technologies.

## Project Overview

This system autonomously collects multiple physiological parameters from a patient, securely transmits the data over Wi-Fi using MQTT protocol, and displays real-time vitals on a cloud-based dashboard for remote clinical oversight. The project demonstrates professional embedded systems development with power management, error handling, and real-time alerting.

### Key Features
- **Multi-Parameter Monitoring**: Heart rate, SpO2, body temperature, ECG, and activity/fall detection
- **Real-time Data Transmission**: Secure MQTT over Wi-Fi with JSON payloads
- **Intelligent Power Management**: Deep sleep modes and battery optimization
- **Cloud Dashboard**: Real-time visualization with Grafana and InfluxDB
- **Advanced Alert System**: Threshold-based critical condition notifications
- **Offline Operation**: Data caching when network is unavailable
- **Professional Codebase**: Modular, maintainable architecture with comprehensive error handling

## System Architecture

## Bill of Materials

| Component | Purpose | Cost | Source |
|-----------|---------|------|--------|
| Arduino Nano ESP32 | Main microcontroller | $25 | Adafruit/Amazon |
| MAX30102 | Heart rate & SpO2 monitoring | $15 | SparkFun |
| AD8232 | Single-lead ECG monitoring | $20 | SparkFun |
| MLX90614 | Non-contact body temperature | $15 | Adafruit |
| MPU6050 | Accelerometer & gyroscope | $3 | Amazon |
| LiPo Battery (1000mAh) | Portable power | $8 | Amazon |
| TP4056 Charger Module | Battery management | $2 | Amazon |
| Breadboard & Wires | Prototyping | $5 | Amazon |

**Total Estimated Cost: ~$93**

## Software Requirements

### Arduino IDE Libraries
```cpp
WiFi.h
PubSubClient.h
Wire.h
ArduinoJson.h
EEPROM.h
SparkFun MAX3010x Pulse and Proximity Sensor Library
SparkFun Bio Sensor Hub Library
Adafruit MLX90614 Library
MPU6050 by Electronic Cats

ESP32 Pin    Sensor Connection
--------    ------------------
3.3V        → ALL: VCC/VIN
GND         → ALL: GND
GPIO 4      → ALL: SDA (I2C)
GPIO 5      → ALL: SCL (I2C)
GPIO 12     → MAX32664: RES_PIN
GPIO 13     → MAX32664: MFIO_PIN
GPIO 14 (A0)→ AD8232: OUTPUT
GPIO 15 (A1)→ AD8232: LO+
GPIO 16 (A2)→ AD8232: LO-

LiPo Battery → TP4056 Charger → ESP32 3.3V
USB Power    →    Module     → System Load
