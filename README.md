# IC-3-Cardiac-Care-Companion
*Honorable Mention award winner of Thailand's NSC competition 2023*

![alt text](https://github.com/ILFforever/IC-3-Cardiac-Care-Companion/blob/main/image/Front.jpg "IC-3 Main Unit")

## Project Overview
- *IC-3* is a compact, portable, and cost-effective smart device designed to monitor heart health and detect potential cardiac issues.
- It integrates **ECG monitoring, pulse rate, and blood oxygen level** measurements into a single device, leveraging AI and algorithms for health analysis and providing accessible data visualization through our web application.

## Features

![alt text](https://github.com/ILFforever/IC-3-Cardiac-Care-Companion/blob/main/image/Graph.png "IC-3 ECG Graph")

### ECG, Heart rate, and SpO2 Measurement.
- ECG and HR measurement using the *uECG sensor*
- Sp02 using the *max30102 sensor*

### Heart Condition Analysis
- AI-Powered Analysis detects abnormalities related to *Atrial Fibrillation (AF)*.
  
![alt text](https://github.com/ILFforever/IC-3-Cardiac-Care-Companion/blob/main/image/AI_detection.png "IC-3 AI Graph")

### Ease of use

![alt text](https://github.com/ILFforever/IC-3-Cardiac-Care-Companion/blob/main/image/App.jpg "IC-3 App")

- Features a compact design with a **built in rechargable battery**.
- Data visualization on our intuitive web application.
- **Touch Screen** for easy interaction on machine.
- Additional face buttons and vibration motor for physical feedback

## Technical Specifications
- Microcontroller: TTGO ESP32 S3 with integrated Bluetooth and Wi-Fi.
- Sensors: Max30102 for SpO2 and pulse rate.
- uECG sensor for ECG readings.
- AI Algorithms: Developed using Python and scikit-learn.
- Cloud Integration: Firebase for data storage and web app connectivity.
- Battery Life: Over 15 hours of continuous usage. (Sensor)

## Installation
- Download and compile the code using either Arduino IDE or PlatformIO (recommended).
> The program is intended to be ran on an ESP32-S3 or ESP32-Wroom 32.
**This program will not function properly on boards without a touchscreen** 
(TTGO S3 Touch Screen version is recomended.)

### Required libraries
```bash
TaskScheduler@^3.7.0
SparkFun MAX3010x Pulse and Proximity Sensor Library@^1.1.2
SdFat - Adafruit Fork@^2.2.1
ArduinoBLE@^1.3.2
Firebase Arduino Client Library for ESP8266 and ESP32@^4.3.8
ArduinoJson@^6.21.1
AsyncTCP@^1.1.1
ESPAsyncWebServer-esphome@^3.0.0
(And other libraries listed in IC-3 Pf-IO/lib)
```

## Acknowledgments
This project is supported by the National Science and Technology Development Agency (NSTDA) and was part of the 25th Thailand Software Development Competition.

## Usage
- Used in conjunction with the uECG Wireless ECG and Heart rate sensor.
  
![alt text](https://cdn.hackaday.io/images/resize/1400x500/1780591562187664851.jpg)

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments
*This project couldn't have been finished without advise and help from the following*
- Asst. Prof. Dr. Ratsameetip Wita 🔥
- Mr. Peerapong Tuptim 🔥
- My Teammates 
- Chiang Mai University for providing support and resources.

## Contact

For inquiries or collaboration, please contact:
Tanabodhi Mukura - Lead Developer
Email: hammymukura@gmail.com

