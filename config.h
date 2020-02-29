/* ----------------- General config -------------------------------- */
/* WiFi */
const char* ssid     = "YOUR SSID";    // Your WiFi SSID
const char* password = "YOUR PSK";     // Your WiFi password

/* MQTT */
const char* mqtt_broker             = "192.168.1.111"; // IP address of your MQTT broker
uint32_t    mqtt_telemetry_period   = 120;             // MQTT report interval in seconds
const char* statusTopic             = "events";        // MQTT topic to report startup

/* Serial */
#define     SERIAL_BAUD_RATE          115200           // Speed for USB serial console
uint32_t    serial_telemetry_period = 10;              // Serial report interval in seconds


/* ----------------- Hardware-specific config ---------------------- */
#define     MODE_BUTTON_PIN           D3               // GPIO0 Pushbutton to GND
#define     PMS_BAUD_RATE             9600             // PMS5003 uses 9600bps
