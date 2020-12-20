/**
  Particulate matter sensor firmware for D1 Mini (ESP8266) and PMS5003

  Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
  Mini (or other ESP8266-based board) and report the values to an MQTT
  broker and to the serial console. Also optionally show them on a 128x32
  I2C OLED display, with a mode button to change between display modes.

  External dependencies. Install using the Arduino library manager:
     "Adafruit GFX Library" by Adafruit
     "Adafruit SSD1306" by Adafruit
     "PubSubClient" by Nick O'Leary
     When using PlatformIO on Visual Studio Code, also add:
     "Adafruit BusIO" by Adafruit

     For BME/BMP280 Sensor
     "Adafruit Unified Sensor" by Adafruit
     "Adafruit BME280 Library" by Adafruit

     For CCS811-Sensor
     "Adafruit CCS811 Library"  by Adafruit

  Bundled dependencies. No need to install separately:
     "PMS Library" by Mariusz Kacki, forked by SwapBap

  Written by Jonathan Oxer for www.superhouse.tv
    https://github.com/superhouse/AirQualitySensorD1Mini

  Inspired by https://github.com/SwapBap/WemosDustSensor/

  Extended by Benjamin Dahlhoff to support BME/BMP280 and more sensors
    https://github.com/bedawi

  Copyright 2020 SuperHouse Automation Pty Ltd www.superhouse.tv
*/
#define VERSION "2.5.1 (bedawi)"
/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <Arduino.h>          // Only needed when using PlatformIO on VSCode
#include <Wire.h>             // For I2C
#include <SoftwareSerial.h>   // Allows PMS to avoid the USB serial port
#include <Adafruit_GFX.h>     // For OLED
#include <Adafruit_SSD1306.h> // For OLED
#include <ESP8266WiFi.h>      // ESP8266 WiFi driver
#include <PubSubClient.h>     // Required for MQTT
#include "PMS.h"              // Particulate Matter Sensor driver (embedded)
#include <Adafruit_Sensor.h>  // Pressure, Temperature, and Humidity
#include <Adafruit_BME280.h>  // Pressure, Temperature, and Humidity Sensor BME/BMP280 connected to i2c-Bus
//#include "Adafruit_CCS811.h"  // Alcohol, aldehydes, ketones, organic acids, amines, aliphatic and aromatic hydrocarbons

/*--------------------------- Classes ------------------------------------*/
// This class provides timer functions for sensors. 
class SensorTimer
{
  int repeat_time;
  int warmup_time;
  uint32_t init_time;
  uint32_t waking_up_since;
  bool readings;

public:
  SensorTimer()
  {
    repeat_time = 120000;
    warmup_time = 60000;
    startover();
    readings = false;
  }
  // Methods
  void startover()
  {
    init_time = millis();
    waking_up_since = 0;
  }

  bool isReady()
  {
    if (millis() >= (init_time + repeat_time))
    {
      if (millis() >= (waking_up_since + warmup_time))
      {
        return true;
      }
    }
    return false;
  }

  bool isTimetoWakeup()
  {
    if (waking_up_since != 0)
    {
      return false;
    }
    if (millis() >= (init_time + repeat_time - warmup_time))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void setRepeatTime(int seconds)
  {
    repeat_time = seconds * 1000;
  }

  void setWarmupTime(int seconds)
  {
    warmup_time = seconds * 1000;
  }

  void skipWaiting()
  {
    init_time = millis() - repeat_time + warmup_time; // Changes the init-time so first measurement starts earlier.
  }

  void wakeUp()
  {
    waking_up_since = millis();
  }

  void readingsTaken()
  {
    readings = true;
  }

  bool readingsWaiting()
  {
    return readings;
  }

  void readingsReported()
  {
    readings = false;
  }
};

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
uint16_t g_pm1p0_sp_value = 0;  // Standard Particle calibration pm1.0 reading
uint16_t g_pm2p5_sp_value = 0;  // Standard Particle calibration pm2.5 reading
uint16_t g_pm10p0_sp_value = 0; // Standard Particle calibration pm10.0 reading

uint16_t g_pm1p0_ae_value = 0;  // Atmospheric Environment pm1.0 reading
uint16_t g_pm2p5_ae_value = 0;  // Atmospheric Environment pm2.5 reading
uint16_t g_pm10p0_ae_value = 0; // Atmospheric Environment pm10.0 reading

uint32_t g_pm0p3_ppd_value = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t g_pm0p5_ppd_value = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t g_pm1p0_ppd_value = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t g_pm2p5_ppd_value = 0;  // Particles Per Deciliter pm2.5 reading
uint32_t g_pm5p0_ppd_value = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t g_pm10p0_ppd_value = 0; // Particles Per Deciliter pm10.0 reading

// BME/BMP280 sensor
float_t g_temperature_celsius_value = 0; // Temperature reading
float_t g_humidity_percent_value = 0; // Humidity reading
float_t g_pressure_hpa_value = 0; // Air pressure reading
float_t g_altitude_meter_value = 0; // Altimeter reading

// MQTT
char g_mqtt_message_buffer[255]; // General purpose buffer for MQTT messages
char g_command_topic[50];        // MQTT topic for receiving commands

#if REPORT_MQTT_SEPARATE
char g_pm1p0_ae_mqtt_topic[50];   // MQTT topic for reporting pm1.0 AE value
char g_pm2p5_ae_mqtt_topic[50];   // MQTT topic for reporting pm2.5 AE value
char g_pm10p0_ae_mqtt_topic[50];  // MQTT topic for reporting pm10.0 AE value
char g_pm0p3_ppd_mqtt_topic[50];  // MQTT topic for reporting pm0.3 PPD value
char g_pm0p5_ppd_mqtt_topic[50];  // MQTT topic for reporting pm0.5 PPD value
char g_pm1p0_ppd_mqtt_topic[50];  // MQTT topic for reporting pm1.0 PPD value
char g_pm2p5_ppd_mqtt_topic[50];  // MQTT topic for reporting pm2.5 PPD value
char g_pm5p0_ppd_mqtt_topic[50];  // MQTT topic for reporting pm5.0 PPD value
char g_pm10p0_ppd_mqtt_topic[50]; // MQTT topic for reporting pm10.0 PPD value
char g_humidity_percent_mqtt_topic[50];
char g_pressure_hpa_mqtt_topic[50];
char g_temperature_celsius_mqtt_topic[50];
char g_altitude_meter_mqtt_topic[50];
#endif
#if REPORT_MQTT_JSON
char g_mqtt_json_topic[50]; // MQTT topic for reporting all values using JSON
#endif

// Weather Sensor (BME/BMP280)
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// Gas Sensor (CCS811)
//Adafruit_CCS811 ccs;

// OLED Display
#define DISPLAY_STATE_GRAMS 1                  // Display values in micrograms/m^3 on screen
#define DISPLAY_STATE_PPD 2                    // Display values in parts per deciliter on screen
#define DISPLAY_STATE_INFO 3                   // Display network status on screen
#define NUM_OF_STATES 3                        // Number of possible states
uint8_t g_display_state = DISPLAY_STATE_GRAMS; // Display values in micrograms/m^3 by default

// Mode Button
uint8_t g_current_mode_button_state = 1; // Pin is pulled high by default
uint8_t g_previous_mode_button_state = 1;
uint32_t g_last_debounce_time = 0;
uint32_t g_debounce_delay = 100;

// Wifi
#define WIFI_CONNECT_INTERVAL 500    // Wait 500ms intervals for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS 10 // Number of attempts/intervals to wait

// General
uint32_t g_device_id; // Unique ID from ESP chip ID

/*--------------------------- Function Signatures ------------------------*/
void mqttCallback(char *topic, byte *payload, uint8_t length);
void checkModeButton();
bool initWifi();
void reconnectMqtt();
void updatePmsReadings();
void reportToMqtt();
void renderScreen();
void reportToSerial();
void updateBMP280Readings();

/*--------------------------- Instantiate Global Objects -----------------*/
// Software serial port
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN); // Rx pin = GPIO2 (D4 on Wemos D1 Mini)

// Particulate matter sensor
PMS pms(pmsSerial); // Use the software serial port for the PMS
PMS::DATA g_data;
SensorTimer pmstimer; // Each sensor can have its own timer

// BME/BMP280 sensor
SensorTimer bmetimer;

// OLED
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MQTT
WiFiClient esp_client;
PubSubClient client(esp_client);

/*--------------------------- Program ------------------------------------*/
/**
  Setup
*/
void setup()
{
  delay(1000); // Short wait - helps debugging on serial console.

  Serial.begin(SERIAL_BAUD_RATE); // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)
  Serial.print("\nAir Quality Sensor starting up, v");
  Serial.println(VERSION);

  //SensorTimer *s1= new SensorTimer();
  //delete s1;
  pmstimer.setRepeatTime(g_pms_report_period);
  pmstimer.setWarmupTime(g_pms_warmup_period);
  pmstimer.skipWaiting();

  // Open a connection to the PMS and put it into passive mode
  pmsSerial.begin(PMS_BAUD_RATE); // Connection for PMS5003
  pms.passiveMode();              // Tell PMS to stop sending data automatically
  delay(100);
  pms.sleep(); // Sending PMS to sleep for now

  // We need a unique device ID for our MQTT client connection
  g_device_id = ESP.getChipId(); // Get the unique ID of the ESP8266 chip
  Serial.print("Device ID: ");
  Serial.println(g_device_id, HEX);

  // Check BME-Sensor
  bmetimer.setRepeatTime(g_bmp_report_period);
  bmetimer.setWarmupTime(0);
  bmetimer.skipWaiting();
  bool status = bme.begin(0x76, &Wire);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  else
  {
    // Setting up BME-Sensor
    Serial.println("BME/BMP280 sensor is online");
    Serial.println();
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
  }

  // Bring CCS811 Sensor Online
  /* if (ccs.begin())
  {
    Serial.println("CCS811 gas sensor is online");
  }
  else
  {
    Serial.println("Failed to start CCS811 gas sensor");
  } */

  // Set up display
  OLED.begin();
  OLED.clearDisplay();
  OLED.setTextWrap(false);
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 0);
  OLED.println("www.superhouse.tv/aqs");
  OLED.println(" Particulate Matter");
  OLED.print(" Sensor v");
  OLED.println(VERSION);
  OLED.print(" Device id: ");
  OLED.println(g_device_id, HEX);
  OLED.display();

  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "tele/d9616f/AE1P0" etc
  sprintf(g_command_topic, "cmnd/%x/COMMAND", ESP.getChipId()); // For receiving commands
#if REPORT_MQTT_SEPARATE
  sprintf(g_pm1p0_ae_mqtt_topic, "tele/%x/AE1P0", ESP.getChipId());     // Data from PMS
  sprintf(g_pm2p5_ae_mqtt_topic, "tele/%x/AE2P5", ESP.getChipId());     // Data from PMS
  sprintf(g_pm10p0_ae_mqtt_topic, "tele/%x/AE10P0", ESP.getChipId());   // Data from PMS
  sprintf(g_pm0p3_ppd_mqtt_topic, "tele/%x/PPD0P3", ESP.getChipId());   // Data from PMS
  sprintf(g_pm0p5_ppd_mqtt_topic, "tele/%x/PPD0P5", ESP.getChipId());   // Data from PMS
  sprintf(g_pm1p0_ppd_mqtt_topic, "tele/%x/PPD1P0", ESP.getChipId());   // Data from PMS
  sprintf(g_pm2p5_ppd_mqtt_topic, "tele/%x/PPD2P5", ESP.getChipId());   // Data from PMS
  sprintf(g_pm5p0_ppd_mqtt_topic, "tele/%x/PPD5P0", ESP.getChipId());   // Data from PMS
  sprintf(g_pm10p0_ppd_mqtt_topic, "tele/%x/PPD10P0", ESP.getChipId()); // Data from PMS

  // Data from BME/BMP280
  sprintf(g_humidity_percent_mqtt_topic, "tele/%x/humidity", ESP.getChipId());
  sprintf(g_pressure_hpa_mqtt_topic, "tele/%x/pressure", ESP.getChipId());
  sprintf(g_temperature_celsius_mqtt_topic, "tele/%x/temperature", ESP.getChipId());
  sprintf(g_altitude_meter_mqtt_topic, "tele/%x/altitude", ESP.getChipId());
#endif
#if REPORT_MQTT_JSON
  sprintf(g_mqtt_json_topic, "tele/%x/SENSOR", ESP.getChipId()); // Data from PMS
#endif

  // Report the MQTT topics to the serial console
  Serial.println(g_command_topic); // For receiving messages
#if REPORT_MQTT_SEPARATE
  Serial.println("MQTT topics:");
  Serial.println(g_pm1p0_ae_mqtt_topic);   // From PMS
  Serial.println(g_pm2p5_ae_mqtt_topic);   // From PMS
  Serial.println(g_pm10p0_ae_mqtt_topic);  // From PMS
  Serial.println(g_pm0p3_ppd_mqtt_topic);  // From PMS
  Serial.println(g_pm0p5_ppd_mqtt_topic);  // From PMS
  Serial.println(g_pm1p0_ppd_mqtt_topic);  // From PMS
  Serial.println(g_pm2p5_ppd_mqtt_topic);  // From PMS
  Serial.println(g_pm5p0_ppd_mqtt_topic);  // From PMS
  Serial.println(g_pm10p0_ppd_mqtt_topic); // From PMS
  // From BME/BMP280
  Serial.println(g_humidity_percent_mqtt_topic);
  Serial.println(g_pressure_hpa_mqtt_topic);
  Serial.println(g_temperature_celsius_mqtt_topic);
  Serial.println(g_altitude_meter_mqtt_topic);
#endif
#if REPORT_MQTT_JSON
  Serial.println(g_mqtt_json_topic); // From PMS
#endif

  // Connect to WiFi
  Serial.println("Connecting to WiFi");
  if (initWifi())
  {
    OLED.println("WiFi [CONNECTED]");
    Serial.println("WiFi connected");
  }
  else
  {
    OLED.println("WiFi [FAILED]");
    Serial.println("WiFi FAILED");
  }
  OLED.display();
  delay(100);

  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP); // Pin for screen mode button

  /* Set up the MQTT client */
  client.setServer(mqtt_broker, 1883);
  client.setCallback(mqttCallback);
  client.setBufferSize(255);
}

/**
  Main loop
*/
void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected())
    {
      reconnectMqtt();
    }
  }
  client.loop(); // Process any outstanding MQTT messages

  checkModeButton();
  updatePmsReadings();
  updateBMP280Readings();
  // renderScreen(); Only render when needed
}

/**
  Read the display mode button and switch the display mode if necessary
*/
void checkModeButton()
{
  g_current_mode_button_state = digitalRead(MODE_BUTTON_PIN);

  // Check if button is now pressed and it was previously unpressed
  if (HIGH == g_previous_mode_button_state && LOW == g_current_mode_button_state)
  {
    // We haven't waited long enough so ignore this press
    if (millis() - g_last_debounce_time <= g_debounce_delay)
    {
      return;
    }
    Serial.println("Button pressed");

    // Increment display state
    g_last_debounce_time = millis();
    if (g_display_state >= NUM_OF_STATES)
    {
      g_display_state = 1;
      return;
    }
    else
    {
      g_display_state++;
      return;
    }
  }

  g_previous_mode_button_state = g_current_mode_button_state;
}

/**
 * Update environment reading vom BME/BMP280
*/

void updateBMP280Readings()
{
  if (bmetimer.isTimetoWakeup())
  {
    // Nothing to do here, sensor does not need to wake up.
    bmetimer.wakeUp();
  }

  if (bmetimer.isReady())
  {
    bme.takeForcedMeasurement();

    // Temperature
    g_temperature_celsius_value = bme.readTemperature();
    // Pressure
    g_pressure_hpa_value = bme.readPressure() / 100.0F;
    // Sealevel (approximation)
    g_altitude_meter_value = bme.readAltitude(SEALEVELPRESSURE_HPA);
    // Humidity
    g_humidity_percent_value = bme.readHumidity();

    bmetimer.readingsTaken();
    reportToMqtt();
    reportToSerial();
    renderScreen();
    bmetimer.readingsReported();
    bmetimer.startover();
  }
}

/**
  Update particulate matter sensor values
*/
void updatePmsReadings()
{
  // Check if we've been in the sleep state for long enough
  if (pmstimer.isTimetoWakeup())
  {
    Serial.println("Waking up PMS sensor");
    pms.wakeUp();
    pmstimer.wakeUp();
  }

  if (pmstimer.isReady())
  {
    Serial.println("PMS sensor is ready. Reading...");
    //pms.requestRead();
    if (pms.readUntil(g_data)) // Use a blocking road to make sure we get values
    {
      g_pm1p0_sp_value = g_data.PM_SP_UG_1_0;
      g_pm2p5_sp_value = g_data.PM_SP_UG_2_5;
      g_pm10p0_sp_value = g_data.PM_SP_UG_10_0;

      g_pm1p0_ae_value = g_data.PM_AE_UG_1_0;
      g_pm2p5_ae_value = g_data.PM_AE_UG_2_5;
      g_pm10p0_ae_value = g_data.PM_AE_UG_10_0;

      // This condition below should NOT be required, but currently I get all
      // 0 values for the PPD results every second time. This check only updates
      // the global values if there is a non-zero result for any of the values:
      if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5 + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5 + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0 != 0)
      {
        g_pm0p3_ppd_value = g_data.PM_TOTALPARTICLES_0_3;
        g_pm0p5_ppd_value = g_data.PM_TOTALPARTICLES_0_5;
        g_pm1p0_ppd_value = g_data.PM_TOTALPARTICLES_1_0;
        g_pm2p5_ppd_value = g_data.PM_TOTALPARTICLES_2_5;
        g_pm5p0_ppd_value = g_data.PM_TOTALPARTICLES_5_0;
        g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
        pms.sleep();
        pmstimer.startover();
        pmstimer.readingsTaken();
      }

      // Report the new values
      reportToMqtt();
      reportToSerial();
      renderScreen();
      pmstimer.readingsReported();
    }
  }
}

/**
  Render the correct screen based on the display mode
*/
void renderScreen()
{
  OLED.clearDisplay();
  OLED.setCursor(0, 0);

  // Render our displays
  switch (g_display_state)
  {
  case DISPLAY_STATE_GRAMS:
    OLED.setTextWrap(false);

    if (pmstimer.readingsWaiting())
    {
      OLED.println("  Particles ug/m^3");

      OLED.print("     PM  1.0: ");
      OLED.println(g_pm1p0_ae_value);

      OLED.print("     PM  2.5: ");
      OLED.println(g_pm2p5_ae_value);

      OLED.print("     PM 10.0: ");
      OLED.println(g_pm10p0_ae_value);
    }
    else
    {
      OLED.println("  Particles ug/m^3");
      OLED.println("  ----------------");
      OLED.println(" Preparing sensor and");
      OLED.println("   waiting for data");
    }
    break;

  case DISPLAY_STATE_PPD:
    OLED.setTextWrap(false);

    if (pmstimer.readingsWaiting())
    {
      OLED.println("Particles / Deciliter");

      OLED.print(" 0.3: ");
      OLED.print(g_pm0p3_ppd_value);
      OLED.setCursor(64, 8);
      OLED.print("0.5:  ");
      OLED.println(g_pm0p5_ppd_value);

      OLED.print(" 1.0: ");
      OLED.print(g_pm1p0_ppd_value);
      OLED.setCursor(64, 16);
      OLED.print("2.5:  ");
      OLED.println(g_pm2p5_ppd_value);

      OLED.print(" 5.0: ");
      OLED.print(g_pm5p0_ppd_value);
      OLED.setCursor(64, 24);
      OLED.print("10.0: ");
      OLED.println(g_pm10p0_ppd_value);
    }
    else
    {
      OLED.println("Particles / Deciliter");
      OLED.println("---------------------");
      OLED.println(" Preparing sensor and");
      OLED.println("   waiting for data");
    }
    break;

  case DISPLAY_STATE_INFO:
    OLED.print("IP:   ");
    OLED.println(WiFi.localIP());
    char mqtt_client_id[20];
    sprintf(mqtt_client_id, "esp8266-%x", g_device_id);
    OLED.setTextWrap(false);
    OLED.print("ID:   ");
    OLED.println(mqtt_client_id);
    OLED.print("SSID: ");
    OLED.println(ssid);
    OLED.print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED)
    {
      OLED.print("OK");
    }
    else
    {
      OLED.print("FAILED");
    }
    OLED.print("   Up:");
    OLED.print((int)millis() / 1000);
    break;

  /* This fallback helps with debugging if you call a state that isn't defined */
  default:
    OLED.println("Unknown state:");
    OLED.println(g_display_state);
    break;
  }

  OLED.display();
}

/**
  Report the latest values to MQTT
*/
void reportToMqtt()
{
  String message_string;

#if REPORT_MQTT_SEPARATE

  if (pmstimer.readingsWaiting())
  {
    /* Report PM1.0 AE value */
    message_string = String(g_pm1p0_ae_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1p0_ae_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM2.5 AE value */
    message_string = String(g_pm2p5_ae_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_ae_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM10.0 AE value */
    message_string = String(g_pm10p0_ae_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10p0_ae_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM0.3 PPD value */
    message_string = String(g_pm0p3_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm0p3_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM0.5 PPD value */
    message_string = String(g_pm0p5_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm0p5_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM1.0 PPD value */
    message_string = String(g_pm1p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1p0_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM2.5 PPD value */
    message_string = String(g_pm2p5_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM5.0 PPD value */
    message_string = String(g_pm5p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm5p0_ppd_mqtt_topic, g_mqtt_message_buffer);

    /* Report PM10.0 PPD value */
    message_string = String(g_pm10p0_ppd_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10p0_ppd_mqtt_topic, g_mqtt_message_buffer);
  }

  if (bmetimer.readingsWaiting())
  {
    message_string = String(g_humidity_percent_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_humidity_percent_mqtt_topic, g_mqtt_message_buffer);

    message_string = String(g_pressure_hpa_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pressure_hpa_mqtt_topic, g_mqtt_message_buffer);

    message_string = String(g_temperature_celsius_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_temperature_celsius_mqtt_topic, g_mqtt_message_buffer);

    message_string = String(g_altitude_meter_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_altitude_meter_mqtt_topic, g_mqtt_message_buffer);
  }
#endif

#if REPORT_MQTT_JSON
  /* Report all values combined into one JSON message */
  // This is an example message generated by Tasmota, to match the format:
  // {"Time":"2020-02-27T03:27:22","PMS5003":{"CF1":0,"CF2.5":1,"CF10":1,"PM1":0,"PM2.5":1,"PM10":1,"PB0.3":0,"PB0.5":0,"PB1":0,"PB2.5":0,"PB5":0,"PB10":0}}
  // This is the source code from Tasmota:
  //ResponseAppend_P(PSTR(",\"PMS5003\":{\"CF1\":%d,\"CF2.5\":%d,\"CF10\":%d,\"PM1\":%d,\"PM2.5\":%d,\"PM10\":%d,\"PB0.3\":%d,\"PB0.5\":%d,\"PB1\":%d,\"PB2.5\":%d,\"PB5\":%d,\"PB10\":%d}"),
  //    pms_g_data.pm10_standard, pms_data.pm25_standard, pms_data.pm100_standard,
  //    pms_data.pm10_env, pms_data.pm25_env, pms_data.pm100_env,
  //    pms_data.particles_03um, pms_data.particles_05um, pms_data.particles_10um, pms_data.particles_25um, pms_data.particles_50um, pms_data.particles_100um);

  // Note: The PubSubClient library limits MQTT message size to 128 bytes. The long format
  // message below only works because the message buffer size has been increased to 255 bytes
  // in setup.

  // BME/BMP280 not yet implemented here!

  // Format the message as JSON in the outgoing message buffer:
  if (pmstimer.readingsWaiting())
  {
    sprintf(g_mqtt_message_buffer, "{\"PMS5003\":{\"CF1\":%i,\"CF1\":%i,\"CF1\":%i,\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i,\"PB0.3\":%i,\"PB0.5\":%i,\"PB1\":%i,\"PB2.5\":%i,\"PB5\":%i,\"PB10\":%i}}",
            g_pm1p0_sp_value, g_pm2p5_sp_value, g_pm10p0_sp_value,
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value,
            g_pm0p3_ppd_value, g_pm0p3_ppd_value, g_pm1p0_ppd_value,
            g_pm2p5_ppd_value, g_pm5p0_ppd_value, g_pm10p0_ppd_value);
  }
  else
  {
    sprintf(g_mqtt_message_buffer, "{\"PMS5003\":{\"CF1\":%i,\"CF1\":%i,\"CF1\":%i,\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i}}",
            g_pm1p0_sp_value, g_pm2p5_sp_value, g_pm10p0_sp_value,
            g_pm1p0_ae_value, g_pm2p5_ae_value, g_pm10p0_ae_value);
  }

  client.publish(g_mqtt_json_topic, g_mqtt_message_buffer);
#endif
}

/**
  Report the latest values to the serial console
*/
void reportToSerial()
{
  if (pmstimer.readingsWaiting())
  {
    /* Report PM1.0 AE value */
    Serial.print("PM1:");
    Serial.println(String(g_pm1p0_ae_value));

    /* Report PM2.5 AE value */
    Serial.print("PM2.5:");
    Serial.println(String(g_pm2p5_ae_value));

    /* Report PM10.0 AE value */
    Serial.print("PM10:");
    Serial.println(String(g_pm10p0_ae_value));
  }

  if (pmstimer.readingsWaiting())
  {
    /* Report PM0.3 PPD value */
    Serial.print("PB0.3:");
    Serial.println(String(g_pm0p3_ppd_value));

    /* Report PM0.5 PPD value */
    Serial.print("PB0.5:");
    Serial.println(String(g_pm0p5_ppd_value));

    /* Report PM1.0 PPD value */
    Serial.print("PB1:");
    Serial.println(String(g_pm1p0_ppd_value));

    /* Report PM2.5 PPD value */
    Serial.print("PB2.5:");
    Serial.println(String(g_pm2p5_ppd_value));

    /* Report PM5.0 PPD value */
    Serial.print("PB5:");
    Serial.println(String(g_pm5p0_ppd_value));

    /* Report PM10.0 PPD value */
    Serial.print("PB10:");
    Serial.println(String(g_pm10p0_ppd_value));
  }

  if (bmetimer.readingsWaiting())
  {
    // Temperature
    Serial.print("Temperature = ");
    Serial.print(g_temperature_celsius_value);
    Serial.println(" *C");

    // Pressure
    Serial.print("Pressure = ");
    Serial.print(g_pressure_hpa_value);
    Serial.println(" hPa");

    // Sealevel (approximation)
    Serial.print("Approx. Altitude = ");
    Serial.print(g_altitude_meter_value);
    Serial.println(" m");

    // Humidity
    Serial.print("Humidity = ");
    Serial.print(g_humidity_percent_value);
    Serial.println(" %");
  }
}

/**
  Connect to Wifi. Returns false if it can't connect.
*/
bool initWifi()
{
  // Clean up any old auto-connections
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.disconnect();
  }
  WiFi.setAutoConnect(false);

  // RETURN: No SSID, so no wifi!
  if (sizeof(ssid) == 1)
  {
    return false;
  }

  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait for connection set amount of intervals
  int num_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && num_attempts <= WIFI_CONNECT_MAX_ATTEMPTS)
  {
    delay(WIFI_CONNECT_INTERVAL);
    num_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
  Reconnect to MQTT broker, and publish a notification to the status topic
*/
void reconnectMqtt()
{
  char mqtt_client_id[20];
  sprintf(mqtt_client_id, "esp8266-%x", ESP.getChipId());

  // Loop until we're reconnected
  while (!client.connected())
  {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password))
    {
      //Serial.println("connected");
      // Once connected, publish an announcement
      sprintf(g_mqtt_message_buffer, "Device %s starting up", mqtt_client_id);
      client.publish(status_topic, g_mqtt_message_buffer);
      // Resubscribe
      //client.subscribe(g_command_topic);
    }
    else
    {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**
  This callback is invoked when an MQTT message is received. It's not important
  right now for this project because we don't receive commands via MQTT. You
  can modify this function to make the device act on commands that you send it.
*/
void mqttCallback(char *topic, byte *payload, uint8_t length)
{
  /*
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    }
    Serial.println();
  */
}
