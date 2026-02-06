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
ESP32 IoT Medical Device
(Sensors + Local Alerts)
        |
        |  MQTT over TLS (8883)
        v
HiveMQ Cloud Broker
(Managed MQTT)
        |
        |  MQTT Subscription
        v
FastAPI Backend (Cloud – Render)
        |
        |  SQLAlchemy ORM
        v
SQLite Time-Series Database
        |
        |  REST API (JSON)
        v
Web Dashboard (HTML/CSS/JS)
(Charts + Live Status)

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

Security Features
-----------------
- Encrypted MQTT communication using TLS
- Username/password authentication
- Local-only dashboard access by default

Data Visualization
------------------
Grafana dashboards provide:
- Real-time ECG heart rate trends
- Battery level visualization
- Historical data analysis
- Threshold-based alerts for abnormal values

Disclaimer
----------
This project is intended for educational and research purposes only and is not
certified for clinical or diagnostic use.

Author
------
Project Name: IoT-Enabled Medical Device for Remote Patient Monitoring
