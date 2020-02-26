/* ----------------- General Config -------------------------------- */
/* WiFi */
const char* ssid     = "YOUR SSID";          // Your WiFi SSID
const char* password = "YOUR PSK";           // Your WiFi password

/* MQTT */
const char* mqtt_broker  = "192.168.1.111";  // IP address of your MQTT broker
const char* statusTopic  = "events";         // MQTT topic to report startup
uint32_t report_interval = 10;              // Report interval in seconds

/* ----------------- Hardware-specific Config ---------------------- */
/* Mode button (momentary between this pin and GND) */
#define MODE_BUTTON_PIN   D7                 // Pushbutton pin

/* Particulate Matter Sensor */
#define PMS_BAUD_RATE   9600
