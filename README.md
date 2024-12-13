Big project so readme edit not done krub
# IC-3-Cardiac-Care-Companion
>"A revolutionary take on Heart desease early detection."

The IC-3 Inteligent Cardiac Care Companion, A Compact ECG and SPO2 Device with Predictive Analytics for Detection of Cardiac Issues, developed under complyment with the NSC competition 2022
>Won the Honorable Mention of Thailand NSC competition 2023
## Installation

Download and run the code using either Arduino IDE or PlatformIO (recommended). The program is intended to be ran on an ESP32-S3 or ESP32-Wroom 32.
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

## Usage

Used in conjunction with the uECG Wireless ECG and Heart rate sensor. For accurate ECG detection.
![alt text](https://cdn.hackaday.io/images/resize/1400x500/1780591562187664851.jpg)

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
