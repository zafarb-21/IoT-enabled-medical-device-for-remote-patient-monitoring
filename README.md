IoT-Enabled Medical Device for Remote Patient Monitoring

Project Overview
----------------
This project presents an IoT-enabled medical device for remote patient monitoring,
designed to continuously collect, transmit, store, and visualize patient vital signs
in real time. The system leverages secure MQTT communication, cloud-based data
ingestion, time-series storage, and interactive dashboards.

The primary objective is to demonstrate an end-to-end IoT healthcare monitoring
pipeline suitable for clinical observation, telemedicine, and research applications.

System Architecture
-------------------
ESP32 Device
   |
   | (MQTT over TLS)
   v
HiveMQ Cloud Broker
   |
   v
Telegraf (MQTT Consumer)
   |
   v
InfluxDB (Time-Series Database)
   |
   v
Grafana (Visualization & Monitoring)

Monitored Parameters
--------------------
- ECG Heart Rate (bpm)
- Heart Rate (bpm)
- Battery Level (%)
- Raw ECG Signal (optional)

Technologies Used
-----------------
Hardware:
- ESP32 Microcontroller
- ECG sensor module
- Power monitoring circuitry

Software:
- MQTT Protocol
- HiveMQ Cloud
- Telegraf
- InfluxDB v2
- Grafana
- Ubuntu Linux

Security Features
-----------------
- Encrypted MQTT communication using TLS
- Username/password authentication
- Token-based access control for InfluxDB
- Local-only dashboard access by default

Data Visualization
------------------
Grafana dashboards provide:
- Real-time ECG heart rate trends
- Battery level visualization
- Historical data analysis
- Threshold-based alerts for abnormal values

Setup Summary
-------------
1. Configure ESP32 to publish sensor data via MQTT
2. Set up HiveMQ Cloud broker with secure credentials
3. Configure Telegraf to subscribe to MQTT topics
4. Store data in InfluxDB bucket (patient_data)
5. Build Grafana dashboards using Flux queries

Applications
------------
- Remote patient monitoring
- Telemedicine systems
- Medical research
- IoT healthcare education
- Wearable healthcare devices

Future Enhancements
-------------------
- Multi-patient support
- Automated alert notifications
- Long-term ECG waveform storage
- Cloud deployment
- Mobile application integration

Disclaimer
----------
This project is intended for educational and research purposes only and is not
certified for clinical or diagnostic use.

Author
------
Project Name: IoT-Enabled Medical Device for Remote Patient Monitoring
