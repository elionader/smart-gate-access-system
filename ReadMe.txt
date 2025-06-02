# Smart Gate Access Control System - README

## Overview

This project implements a secure and automated smart gate system using Raspberry Pi, license plate recognition, and OTP-based guest access. It includes three roles: Admin, Family Members, and Guest.

## Hardware Requirements

* Raspberry Pi 3B+
* Stepper Motor with ULN2003 driver
* Breadboard and jumper wires
* PiCamera
* 5V Power Adapter
* Internet access (Wi-Fi or Ethernet)

## Software Requirements

* Raspberry Pi OS (Buster Lite recommended)
* Python 3
* Firebase Admin SDK
* Google Cloud Vision API key
* MQTT via HiveMQ Cloud

## Setup Instructions

### 1. Hardware Wiring

* Connect ULN2003 IN1-4 to GPIO pins (e.g., GPIO17, GPIO18, GPIO27, GPIO22)
* Connect stepper motor to ULN2003
* Connect 5V external power to ULN2003
* Connect PiCamera to CSI port

### 2. Firebase

* Create a Firebase Realtime Database
* Download and place `firebase_key.json` in the same directory
* Structure: `/users/{username}/plates: ["ABC123", "XYZ789"]`

### 3. Google Cloud Vision

* Enable Vision API from Google Cloud Console
* Get API key and add it in `CONFIG["google_vision"]["api_key"]`

### 4. MQTT

* Use HiveMQ or similar broker
* Setup topics in the MQTT broker (use config structure)

### 5. Running the System

```bash
# Install dependencies
sudo apt update && sudo apt upgrade -y
pip install paho-mqtt firebase-admin opencv-python numpy requests

# Run the main script
python3 mqtt_pi_client.py
```

## MQTT Topics Summary

### Commands (sent to Pi)

* `gate/command/open` → Manually open gate
* `gate/command/generate_otp` → Generate OTP for guest
* `gate/command/verify_otp` → Verify OTP from guest
* `gate/command/generate_report` → Trigger weekly report
* `gate/command/camera` → Start/stop camera stream
* `gate/command/detect_plate` → Detect license plate

### Status (published by Pi)

* `gate/status/entry` → Entry log messages
* `gate/status/plate` → Detected license plate
* `gate/status/otp` → OTP generated
* `gate/status/report` → Weekly report update
* `gate/status/camera` → Streaming status
* `gate/system/status` → Uptime and online/offline
* `gate/security/alerts` → Unauthorized or invalid attempts

## Features Summary

* **Admin**: Add/remove users and plates, view logs, generate reports
* **Family**: Access with plate recognition
* **Guest**: Access with OTP

## Notes

* Ensure camera permissions and interface are enabled on the Pi.
* You can access the MJPG stream from: `http://<Pi-IP>:8080/?action=stream`
* All sensors and motors must be properly grounded.

## Authors & Acknowledgements

Project developed by: Elio Nader and Georges Haddad
Special thanks to OpenAI and SeeedStudio for documentation references.
