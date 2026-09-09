# Fall Detection and Emergency Alert System

An ESP32-based IoT safety and emergency alert system designed to detect falls, monitor vital signs, obtain GPS location, and automatically notify an emergency contact through SMS.

The system combines motion sensing, physiological monitoring, GPS tracking, Wi-Fi connectivity, and an emergency alert mechanism into a single embedded platform.

Watch System testing video
https://lnkd.in/p/gRB75w8P 
Or Profile of https://www.linkedin.com/in/md-saihan-alam

---

## Features

- Automatic fall detection using MPU6050
- Free-fall detection
- Impact detection
- Multiple-impact detection
- Audible emergency alarm using buzzer
- Manual emergency alert button
- Heart-rate monitoring using MAX30102
- SpO2 monitoring using MAX30102
- Temperature monitoring
- GPS location tracking using NEO-6M
- Wi-Fi connectivity using ESP32
- Emergency SMS notification through TextBEE API
- Google Maps location link in emergency SMS
- GPS satellite and altitude information
- SMS retry mechanism
- LED-based system status indication
- Automatic alarm cancellation
- Automatic system reset

---

## System Overview

The system continuously monitors the user's movement using the MPU6050 accelerometer.

When a possible fall is detected, the ESP32 activates a local alarm. The user can cancel the alarm using the push button.

If the alarm remains active and the device appears to be stationary, the system obtains the latest GPS information and vital signs before sending an emergency SMS.

### System Flow

```text
                 ┌─────────────────┐
                 │     MPU6050     │
                 │ Motion Sensor   │
                 └────────┬────────┘
                          │
                          ▼
                  ┌───────────────┐
                  │ Fall Detection│
                  └───────┬───────┘
                          │
                    Fall detected
                          │
                          ▼
                  ┌───────────────┐
                  │     ESP32     │
                  │ Main Control  │
                  └───────┬───────┘
                          │
             ┌────────────┼─────────────┐
             │            │             │
             ▼            ▼             ▼
        ┌─────────┐  ┌──────────┐  ┌──────────┐
        │MAX30102 │  │ NEO-6M   │  │  Buzzer  │
        │HR/SpO2  │  │   GPS    │  │  + LED   │
        └─────────┘  └──────────┘  └──────────┘
             │            │
             └────────────┤
                          ▼
                    ┌────────────┐
                    │   Wi-Fi    │
                    └─────┬──────┘
                          │
                          ▼
                    ┌────────────┐
                    │ TextBEE API│
                    └─────┬──────┘
                          │
                          ▼
                   Emergency SMS