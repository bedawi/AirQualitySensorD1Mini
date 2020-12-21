# Air Quality and Environment Sensor Firmware for ESP8266

## Forked from superhouse.tv
This fork from supports more than one sensor. In addition to the PMS5003 the BME/BMP280 sensor has been added to measure temperatur, humidity and pressure. Please check out thir blog for detailed information on their projects: https://www.superhouse.tv/

## What does it do?

Read from a Plantower PMS5003 particulate matter sensor and the BME/BMP280 environment sensor using a Wemos D1 Mini (or other ESP8266-based board) and report the values to an MQTT broker and show them on an OLED display.

The mode button and OLED are optional. You can build this device without display or button, and it will work just fine. The serial console provides insights on whats happening under the hood.

On startup, the device reads its own ESP8266 chip ID and uses this as the basis for its MQTT topics and client ID, to ensure that they will be unique for each device.

It displays this ID on the OLED display so you can read it off the display if you have one installed. In case you don't, it also reports the ID via the serial console at 9600bps, and publishes a startup message to the topic "events" including its own ID. You can watch that topic while the device is booting to see what ID it reports.

This ID is then used to generate topics for it to report its readings.
The topics take the following form, with its unique ID substituted:

 * tele/d9616f/PM1
 * tele/d9616f/PM2.5
 * tele/d9616f/PM10
 * tele/d9616f/PPD0.3
 * tele/d9616f/PPD0.5
 * ...etc

The values are also reported in a combined format as JSON, in the same
structure used by Tasmota, at a topic similar to:

 * tele/d9616f/SENSOR

If you install the OLED and the mode button, you can use the button to toggle the display between different screens. The first screen shows the most recent readings in micrograms per cubic meter. The second screen shows the most recent readings in particles per deciliter. The third screen shows network information including the MQTT ID, IP address, WiFi SSID, WiFi connection status, and uptime.

## Dependencies

The source code has been changed to work in Visual Studio Code with PlatformIO Extension installed. It is possibly no longer fully compatible with the Arduino IDE.

Once imported into a PlatformIO project the following libraries have to be added:

* "Adafruit GFX Library" by Adafruit
* "Adafruit SSD1306" by Adafruit
* "PubSubClient" by Nick O'Leary
* "Adafruit BusIO" by Adafruit
* "Adafruit Unified Sensor" by Adafruit
* "Adafruit BME280 Library" by Adafruit

It also uses a version of Mariusz Kacki's "PMS" library that was forked by SwapBap. You do not need to install this library separately, because it's included in the project.

## Connections

For particulate matter sensor:
 * PMS5003 pin 1 (VCC) to D1 Mini "5V" pin
 * PMS5003 pin 2 (GND) to D1 Mini "GND" pin
 * PMS5003 pin 4 (RX) to D1 Mini "D7" pin (Different from original project!)
 * PMS5003 pin 5 (TX) to D1 Mini "D6" pin (Different from original project!)

For mode button:
 * Button connected between D1 Mini "D7" pin (GPIO13) and GND

The OLED and the BME/BMP280 sensor via I2C
 * OLED VCC to D1 Mini 3.3V
 * OLED GND to D1 Mini GND
 * OLED SCL to D1 Mini "D1" pin
 * OLED SDA to D1 Mini "D2" pin

## To do

Open/planned:
* Better Display output and automatic cycle.
* More Sensors and option to enable and disable these in config.h
* Interactive WiFi-Setup (possibly needs better Hardware...).

## Changes from original project:

* Indroduced a timer class to manage timing for more then one sensor.
* Included BME/BMP280 sensor and its values for humidity, temperature, pressure and altitude.
* Renamed config.h into config_sample.h and put config.h to .gitignore file.

## Credits

Written by Jonathan Oxer for www.superhouse.tv  
https://github.com/superhouse/AirQualitySensorD1Mini

Inspired by https://github.com/SwapBap/WemosDustSensor

Forked and extended by Benjamin Dahlhoff.
https://github.com/bedawi/AirQualitySensorD1Mini