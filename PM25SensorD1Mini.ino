/**
  Particulate matter sensor firmware for D1 Mini / ESP8266
  Version: 2.1

  By Jonathan Oxer for www.superhouse.tv

  Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
  Mini (or other ESP8266-based board) and report the values to an MQTT
  broker and to the serial console. Also optionally show them on a 128x32
  I2C OLED display, with a mode button to change between display modes.

  External dependencies, all available in the Arduino library manager:
     "Adafruit GFX Library" by Adafruit
     "Adafruit SSD1306" by Adafruit
     "PMS Library" by Mariusz Kacki
     "PubSubClient" by Nick O'Leary

  More information: https://github.com/superhouse/PM25SensorD1Mini

  Inspired by: https://github.com/SwapBap/WemosDustSensor/
*/

/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <SoftwareSerial.h>             // Allows PMS to avoid the USB serial port
#include <Adafruit_GFX.h>               // Required for OLED
#include <Adafruit_SSD1306.h>           // Required for OLED
#include <ESP8266WiFi.h>                // ESP8266 WiFi driver
#include <PMS.h>                        // Particulate Matter Sensor driver
#include <PubSubClient.h>               // Required for MQTT

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
#define SAMPLE_COUNT 50                 // Number of samples to average
uint8_t g_pm1_latest_value   = 0;       // Latest pm1.0 reading from sensor
uint8_t g_pm2p5_latest_value = 0;       // Latest pm2.5 reading from sensor
uint8_t g_pm10_latest_value  = 0;       // Latest pm10.0 reading from sensor
uint16_t g_pm1_ring_buffer[SAMPLE_COUNT];   // Ring buffer for averaging pm1.0 values
uint16_t g_pm2p5_ring_buffer[SAMPLE_COUNT]; // Ring buffer for averaging pm2.5 values
uint16_t g_pm10_ring_buffer[SAMPLE_COUNT];  // Ring buffer for averaging pm10.0 values
uint8_t  g_ring_buffer_index = 0;       // Current position in the ring buffers
uint32_t g_pm1_average_value   = 0;     // Average pm1.0 reading from buffer
uint32_t g_pm2p5_average_value = 0;     // Average pm2.5 reading from buffer
uint32_t g_pm10_average_value  = 0;     // Average pm10.0 reading from buffer

// MQTT
String g_device_id = "default";         // This is replaced later with a unique value
uint32_t g_last_mqtt_report_time = 0;   // Timestamp of last report to MQTT
char g_mqtt_message_buffer[150];        // General purpose buffer for MQTT messages
char g_command_topic[50];               // MQTT topic for receiving commands
#if REPORT_MQTT_TOPICS
char g_pm1_mqtt_topic[50];              // MQTT topic for reporting averaged pm1.0 values
char g_pm2p5_mqtt_topic[50];            // MQTT topic for reporting averaged pm2.5 values
char g_pm10_mqtt_topic[50];             // MQTT topic for reporting averaged pm10.0 values
char g_pm1_raw_mqtt_topic[50];          // MQTT topic for reporting raw pm1.0 values
char g_pm2p5_raw_mqtt_topic[50];        // MQTT topic for reporting raw pm2.5 values
char g_pm10_raw_mqtt_topic[50];         // MQTT topic for reporting raw pm10.0 values
#endif
#if REPORT_MQTT_JSON
char g_mqtt_json_topic[50];             // MQTT topic for reporting all values in JSON
#endif

// Serial
uint32_t g_last_serial_report_time = 0; // Timestamp of last report to serial console

// OLED Display
#define STATE_GRAMS   1                 // Display PMS values in grams on screen
#define STATE_INFO    2                 // Display network status on screen
#define NUM_OF_STATES 2                 // Number of possible states
uint8_t g_display_state = STATE_GRAMS;  // Display values in grams by default

// Mode Button
uint8_t  g_current_mode_button_state  = 1;   // Pin is pulled high by default
uint8_t  g_previous_mode_button_state = 1;
uint32_t g_last_debounce_time         = 0;
uint32_t g_debounce_delay             = 100;

// Wifi
#define WIFI_CONNECT_INTERVAL 500       // Wait 500ms intervals for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS 10    // Number of attempts/intervals to wait

/*--------------------------- Function Signatures ---------------------------*/
void mqttCallback(char* topic, byte* payload, uint8_t length);
void checkModeButton();
bool initWifi();
void reconnectMqtt();
void updatePmsReadings();
void reportToMqtt();
void renderScreen();

/*--------------------------- Instantiate Global Objects --------------------*/
// Software serial port
SoftwareSerial pmsSerial(D4, NULL);     // Rx pin = GPIO2 (D4 on Wemos D1 Mini)

// Particulate matter sensor
PMS pms(pmsSerial);                     // Use the software serial port for the PMS
PMS::DATA data;

// OLED
Adafruit_SSD1306 OLED(0);               // GPIO0 = OLED reset pin

// MQTT
WiFiClient esp_client;
PubSubClient client(esp_client);

/*--------------------------- Program ---------------------------------------*/
/**
  Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);   // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)
  Serial.println();
  Serial.println("Air Quality Sensor starting up, v2.1");

  pmsSerial.begin(PMS_BAUD_RATE);   // Connection for PMS5003

  // We need a unique device ID for our MQTT client connection
  g_device_id = String(ESP.getChipId(), HEX);  // Get the unique ID of the ESP8266 chip in hex
  Serial.print("Device ID: ");
  Serial.println(g_device_id);

  // Set up display
  OLED.begin();
  OLED.clearDisplay();
  OLED.setTextWrap(false);
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 0);
  OLED.println(" www.superhouse.tv");
  OLED.println(" Particulate Matter");
  OLED.println(" Sensor v2.1");
  OLED.print  (" Device id: ");
  OLED.println(g_device_id);
  OLED.display();

  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "device/d9616f/pm1" etc
  sprintf(g_command_topic,        "cmnd/%x/COMMAND",  ESP.getChipId());  // For receiving commands
#if REPORT_MQTT_TOPICS
  sprintf(g_pm1_mqtt_topic,       "tele/%x/PM1",      ESP.getChipId());  // Data from PMS
  sprintf(g_pm2p5_mqtt_topic,     "tele/%x/PM2P5",    ESP.getChipId());  // Data from PMS
  sprintf(g_pm10_mqtt_topic,      "tele/%x/PM10",     ESP.getChipId());  // Data from PMS
  sprintf(g_pm1_raw_mqtt_topic,   "tele/%x/PM1RAW",   ESP.getChipId());  // Data from PMS
  sprintf(g_pm2p5_raw_mqtt_topic, "tele/%x/PM2P5RAW", ESP.getChipId());  // Data from PMS
  sprintf(g_pm10_raw_mqtt_topic,  "tele/%x/PM10RAW",  ESP.getChipId());  // Data from PMS
#endif
#if REPORT_MQTT_JSON
  sprintf(g_mqtt_json_topic,      "tele/%x/SENSOR",   ESP.getChipId());  // Data from PMS
#endif

  // Report the MQTT topics to the serial console
  Serial.println(g_command_topic);          // For receiving messages
#if REPORT_MQTT_TOPICS
  Serial.println("MQTT topics:");
  Serial.println(g_pm1_mqtt_topic);         // From PMS
  Serial.println(g_pm2p5_mqtt_topic);       // From PMS
  Serial.println(g_pm10_mqtt_topic);        // From PMS
  Serial.println(g_pm1_raw_mqtt_topic);     // From PMS
  Serial.println(g_pm2p5_raw_mqtt_topic);   // From PMS
  Serial.println(g_pm10_raw_mqtt_topic);    // From PMS
#endif
#if REPORT_MQTT_JSON
  Serial.println(g_mqtt_json_topic);        // From PMS
#endif

  // Connect to WiFi
  if (initWifi())
  {
    OLED.println("Wifi [CONNECTED]");
  } else {
    OLED.println("Wifi [FAILED]");
  }
  OLED.display();
  delay(1000);

  pinMode(MODE_BUTTON_PIN , INPUT_PULLUP); // Pin for switching screens button

  /* Set up the MQTT client */
  client.setServer(mqtt_broker, 1883);
  client.setCallback(mqttCallback);
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
  client.loop();  // Process any outstanding MQTT messages

  checkModeButton();
  updatePmsReadings();
  renderScreen();
  reportToMqtt();
  reportToSerial();
  updateAverages();
}

/**
  Render the correct screen based on the display state
*/
void renderScreen()
{
  OLED.clearDisplay();
  OLED.setCursor(0, 0);

  // Render our displays
  switch (g_display_state)
  {
    case STATE_GRAMS:
      OLED.setTextWrap(false);
      OLED.println("  Particles ug/m^3");
      OLED.print("     PM  1.0: ");
      OLED.println(data.PM_AE_UG_1_0);
      //OLED.println("ug/m3");

      OLED.print("     PM  2.5: ");
      OLED.println(data.PM_AE_UG_2_5);
      //OLED.println("ug/m3");

      OLED.print("     PM 10.0: ");
      OLED.println(data.PM_AE_UG_10_0);
      //OLED.println("ug/m3");

      break;

    case STATE_INFO:
      OLED.print("IP:   ");
      OLED.println(WiFi.localIP());
      char mqtt_client_id[20];
      sprintf(mqtt_client_id, "esp8266-%x", ESP.getChipId());
      OLED.setTextWrap(false);
      OLED.print("ID:   ");
      OLED.println(mqtt_client_id);
      OLED.print("SSID: ");
      OLED.println(ssid);
      OLED.print("WiFi: ");
      if (WiFi.status() == WL_CONNECTED)
      {
        OLED.println("Connected");
      } else {
        OLED.println("FAILED");
      }
      break;

    /* This fallback helps with debugging if you call a state that isn't defined */
    default:
      OLED.println(g_display_state);
      break;
  }

  OLED.display();
}

/**
  Read the display mode button and switch the display mode if necessary
*/
void checkModeButton()
{
  g_previous_mode_button_state = g_current_mode_button_state;
  g_current_mode_button_state = digitalRead(MODE_BUTTON_PIN);

  // Check if button is now pressed and it was previously unpressed
  if (g_current_mode_button_state == LOW && g_previous_mode_button_state == HIGH)
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
    } else {
      g_display_state++;
      return;
    }
  }
}

/**
  Update particulate matter sensor values
*/
void updatePmsReadings()
{
  if (pms.read(data))
  {
    g_pm1_latest_value   = data.PM_AE_UG_1_0;
    g_pm2p5_latest_value = data.PM_AE_UG_2_5;
    g_pm10_latest_value  = data.PM_AE_UG_10_0;

    g_pm1_ring_buffer[g_ring_buffer_index]   = g_pm1_latest_value;
    g_pm2p5_ring_buffer[g_ring_buffer_index] = g_pm2p5_latest_value;
    g_pm10_ring_buffer[g_ring_buffer_index]  = g_pm10_latest_value;
    g_ring_buffer_index++;
    if (g_ring_buffer_index > SAMPLE_COUNT)
    {
      g_ring_buffer_index = 0;
    }
  }
}

/**
  Calculates averages for the samples in the ring buffers and stores them
  in global variables for reference by the reporting functions.
*/
void updateAverages()
{
  uint8_t i;
  for (i = 0; i < SAMPLE_COUNT; i++)
  {
    g_pm1_average_value += g_pm1_ring_buffer[i];
  }
  g_pm1_average_value = (int)((g_pm1_average_value / SAMPLE_COUNT) + 0.5);

  for (i = 0; i < SAMPLE_COUNT; i++)
  {
    g_pm2p5_average_value += g_pm2p5_ring_buffer[i];
  }
  g_pm2p5_average_value = (int)((g_pm2p5_average_value / SAMPLE_COUNT) + 0.5);

  //for (i = 0; i < sizeof(g_pm10_ring_buffer) / sizeof( g_pm10_ring_buffer[0]); i++)
  for (i = 0; i < SAMPLE_COUNT; i++)
  {
    g_pm10_average_value += g_pm10_ring_buffer[i];
  }
  g_pm10_average_value = (int)((g_pm10_average_value / SAMPLE_COUNT) + 0.5);
}

/**
  Report the most recent values to MQTT if enough time has passed
*/
void reportToMqtt()
{
  uint32_t time_now = millis();
  if (time_now - g_last_mqtt_report_time > (mqtt_telemetry_period * 1000))
  {
    g_last_mqtt_report_time = time_now;
    updateAverages();

    String message_string;

#if REPORT_MQTT_TOPICS
    /* Report averaged PM1 value */
    message_string = String(g_pm1_average_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_mqtt_topic, g_mqtt_message_buffer);

    /* Report averaged PM2.5 value */
    message_string = String(g_pm2p5_average_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_mqtt_topic, g_mqtt_message_buffer);

    /* Report averaged PM10 value */
    message_string = String(g_pm10_average_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM1 value for comparison with averaged value */
    message_string = String(g_pm1_latest_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm1_raw_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM2.5 value for comparison with averaged value */
    message_string = String(g_pm2p5_latest_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm2p5_raw_mqtt_topic, g_mqtt_message_buffer);

    /* Report raw PM10 value for comparison with averaged value */
    message_string = String(g_pm10_latest_value);
    message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    client.publish(g_pm10_raw_mqtt_topic, g_mqtt_message_buffer);
#endif
#if REPORT_MQTT_JSON
    /* Report all values combined into one JSON message */
    //message_string = String(g_pm10_latest_value);
    //message_string.toCharArray(g_mqtt_message_buffer, message_string.length() + 1);
    //client.publish(g_mqtt_json_topic, g_mqtt_message_buffer);
    // {"Time":"2020-02-27T03:27:22","PMS5003":{"CF1":0,"CF2.5":1,"CF10":1,"PM1":0,"PM2.5":1,"PM10":1,"PB0.3":0,"PB0.5":0,"PB1":0,"PB2.5":0,"PB5":0,"PB10":0}}
    sprintf(g_mqtt_message_buffer,  "{\"PMS5003\":{\"CF1\":0,\"CF2.5\":0,\"CF10\":0,\"PM1\":%i,\"PM2.5\":%i,\"PM10\":%i,\"PB0.3\":0,\"PB0.5\":0,\"PB1\":0,\"PB2.5\":0,\"PB5\":0,\"PB10\":0}}",
        g_pm1_latest_value, g_pm2p5_latest_value, g_pm10_latest_value);
    client.publish(g_mqtt_json_topic, g_mqtt_message_buffer);
    //ResponseAppend_P(PSTR(",\"PMS5003\":{\"CF1\":%d,\"CF2.5\":%d,\"CF10\":%d,\"PM1\":%d,\"PM2.5\":%d,\"PM10\":%d,\"PB0.3\":%d,\"PB0.5\":%d,\"PB1\":%d,\"PB2.5\":%d,\"PB5\":%d,\"PB10\":%d}"),
    //    pms_data.pm10_standard, pms_data.pm25_standard, pms_data.pm100_standard,
    //    pms_data.pm10_env, pms_data.pm25_env, pms_data.pm100_env,
    //    pms_data.particles_03um, pms_data.particles_05um, pms_data.particles_10um, pms_data.particles_25um, pms_data.particles_50um, pms_data.particles_100um);
#endif
  }
}

/**
  Report the most recent values to the serial console if enough time has passed
*/
void reportToSerial()
{
  uint32_t time_now = millis();
  if (time_now - g_last_serial_report_time > (serial_telemetry_period * 1000))
  {
    g_last_serial_report_time = time_now;
    updateAverages();

    String message_string;

    /* Report averaged PM1 value */
    message_string = String(g_pm1_average_value);
    Serial.print("PM1_AVERAGE:");
    Serial.println(message_string);

    /* Report averaged PM2.5 value */
    message_string = String(g_pm2p5_average_value);
    Serial.print("PM2P5_AVERAGE:");
    Serial.println(message_string);

    /* Report averaged PM10 value */
    message_string = String(g_pm10_average_value);
    Serial.print("PM10_AVERAGE:");
    Serial.println(message_string);

    /* Report raw PM1 value for comparison with averaged value */
    message_string = String(g_pm1_latest_value);
    Serial.print("PM1_LATEST:");
    Serial.println(message_string);

    /* Report raw PM2.5 value for comparison with averaged value */
    message_string = String(g_pm2p5_latest_value);
    Serial.print("PM2P5_LATEST:");
    Serial.println(message_string);

    /* Report raw PM10 value for comparison with averaged value */
    message_string = String(g_pm10_latest_value);
    Serial.print("PM10_LATEST:");
    Serial.println(message_string);
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
  } else {
    return true;
  }
}

/**
  Reconnect to MQTT broker, and publish a notification to the status topic
*/
void reconnectMqtt() {
  char mqtt_client_id[20];
  sprintf(mqtt_client_id, "esp8266-%x", ESP.getChipId());

  // Loop until we're reconnected
  while (!client.connected())
  {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_id))
    {
      //Serial.println("connected");
      // Once connected, publish an announcement
      sprintf(g_mqtt_message_buffer, "Device %s starting up", mqtt_client_id);
      client.publish(statusTopic, g_mqtt_message_buffer);
      // Resubscribe
      //client.subscribe(g_command_topic);
    } else {
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
void mqttCallback(char* topic, byte* payload, uint8_t length)
{
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //for (int i = 0; i < length; i++) {
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();
}
