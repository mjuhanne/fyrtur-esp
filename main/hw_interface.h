/*

	Hardware interface for controlling blinds:

	UP/DOWN BUTTONS:  
		Single click: Starts rolling blinds up/down if they are stopped. Otherwise stops movement. LED will blink once.
		Double click: 
			- Set lower curtain limit to the current position (curtains must be stopped first). LED will blink twice.
			- If curtains are at top position, lower curtain limit will be reset to the full length. LED will blink three times.

	UP or DOWN BUTTON held down for 1 second: Continous slow movement up/down overriding the previously set max/full curtain length limits.
		- Setting the custom speed is not possible in original motor firmware. Movement will be done in small steps instead.

	Both buttons are held down:
		2000 milliseconds : start Wireless Access point
		3500 ms : led starts blinking, warning about imminent factory reset
		6000 ms : do a factory reset:
			- LED will stay on
			- NVS partition is formatted and Variables are reset to default values
			- ESP module is reseted
			- Wireless Access Point is restarted in order to re-configure WiFi and MQTT 

	LED: 
		- no blinking: in standby
		- 1000ms on and off: WiFi needs configration (please connect to Fyrtur AP)
		- 2 short blinks repeating: Connecting to AP, or WiFI is in disconnected state
		- 3 short blinks repeating: Connecting to MQTT server, or in disconnected state
		- 2 sec blink: MQTT connected

*/
#include "sdkconfig.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#define ESP32
#endif


#define SENSOR_TASK_CORE 1

#define STATUS_LED 0

#define SLOW_MOVEMENT_SPEED	6	// in rpm

#define MAX_BUTTONS 2
#define BTN_1 0  // "UP" button
#define BTN_2 1 // "DOWN" button

// --- Variables for handling keeping both buttons pressed
#define START_AP_THRESHOLD           2000
#define FACTORY_RESET_IMMINENT_THRESHOLD 3500
#define FACTORY_RESET_THRESHOLD          5000

#define DOUBLE_CLICK_PERIOD 500 // milliseconds. Two button clicks during this period of time is considered as double-click
#define LONG_PRESS_PERIOD 1000 // milliseconds. A button press longer than this is considered as a long press

#define WIFI_LED_PRIORITY 0
#define MQTT_LED_PRIORITY 0
#define OTA_LED_PRIORITY 10
#define FACTORY_RESET_LED_PRIORITY 20
#define RESET_LED_PRIORITY 5
#define BTN_LED_PRIORITY 0
#define NORMAL_LED_PRIORITY 0

#define IOT_BTN_LED_COLOR 50,50,50
#define IOT_WIFI_LED_COLOR 0,0,30
#define IOT_MQTT_LED_COLOR 30,0,30
#define IOT_OTA_LED_COLOR 30,0,0
#define IOT_FACTORY_RESET_LED_COLOR 0,30,0


#define IOT_UNCONFIGURED_LED() iot_led_pulse(STATUS_LED, IOT_WIFI_LED_COLOR, 1000, 1000, -1, NORMAL_LED_PRIORITY);
#define IOT_WIFI_DISCONNECTED_LED() iot_led_burst(STATUS_LED, IOT_WIFI_LED_COLOR, 50, 300, 2, -1, 1000, WIFI_LED_PRIORITY);
#define IOT_WIFI_CONNECTING_LED() iot_led_burst(STATUS_LED, IOT_WIFI_LED_COLOR, 50, 300, 2, -1, 1000, WIFI_LED_PRIORITY);
#define IOT_WIFI_CONNECTED_LED() ;	// ignored, since MQTT connection attempt is started right after this event
#define IOT_MQTT_CONNECTING_LED() iot_led_burst(STATUS_LED, IOT_MQTT_LED_COLOR, 50, 300, 3, -1, 1000, MQTT_LED_PRIORITY);
#define IOT_MQTT_CONNECTED_LED() iot_led_blink(STATUS_LED, IOT_MQTT_LED_COLOR, 2000, MQTT_LED_PRIORITY);
#define IOT_MQTT_DISCONNECTED_LED() iot_led_burst(STATUS_LED, IOT_MQTT_LED_COLOR, 50, 300, 3, -1, 1000, MQTT_LED_PRIORITY);

#define IOT_LED_OFF() iot_led_set(STATUS_LED, 0, 0, 0);
#define IOT_LED_ON() iot_led_set(STATUS_LED, 50, 0, 0);

#define IOT_FACTORY_RESET_LED() iot_led_set_priority(STATUS_LED, IOT_FACTORY_RESET_LED_COLOR, FACTORY_RESET_LED_PRIORITY);
#define IOT_FACTORY_RESET_IMMINENT_LED() iot_led_pulse(STATUS_LED, IOT_FACTORY_RESET_LED_COLOR, 50, 50, -1, FACTORY_RESET_LED_PRIORITY );

#define IOT_START_AP_LED() iot_led_pulse(STATUS_LED, IOT_BTN_LED_COLOR, 100, 100, 4, NORMAL_LED_PRIORITY );

#define IOT_SET_LIMITS_LED() iot_led_pulse(STATUS_LED, IOT_BTN_LED_COLOR, 100, 100, 2, NORMAL_LED_PRIORITY );
#define IOT_RESET_LIMITS_LED() iot_led_pulse(STATUS_LED, IOT_BTN_LED_COLOR, 100, 100, 3, NORMAL_LED_PRIORITY );

#define IOT_SHORT_CLICK_LED() iot_led_blink(STATUS_LED, IOT_BTN_LED_COLOR, 100, BTN_LED_PRIORITY);

#define IOT_OTA_STARTING() iot_led_set_priority(STATUS_LED, 30, 0, 30, OTA_LED_PRIORITY);

#define IOT_OTA_FAILED() iot_led_burst(STATUS_LED, 30, 0, 0, 300, 300, 5, 3, 1000, 0);

#define IOT_MQTT_MSG_OK_LED() iot_led_blink(STATUS_LED, IOT_MQTT_LED_COLOR, 100, MQTT_LED_PRIORITY);
#define IOT_MQTT_MSG_ERROR_LED() iot_led_pulse(STATUS_LED, IOT_MQTT_LED_COLOR, 100, 100, 2, MQTT_LED_PRIORITY );




#ifdef ESP32
#define BTN1_GPIO 21 // Btn up
#define BTN2_GPIO 22 // Btn down
#define LED_GPIO 18
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 19

// Motor unit UART pins
#define TXD_PIN (GPIO_NUM_23)	// TX pin to Motor unit
#define RXD_PIN (GPIO_NUM_5)	// RX pin from Motor unit

// these are for debugging purposes and not normally connected. OTA updates should work without these if motor FW version is >0.8 
#define RESET_PIN (GPIO_NUM_19)	
#define BOOT0_PIN (GPIO_NUM_4) 

#else	// ESP8266
#define BTN1_GPIO 14 // Btn up
#define BTN2_GPIO 0 // Btn down (doubles as download/flash button)
#define LED_GPIO 12
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 5

// Motor unit UART pins
//#define TXD_PIN (GPIO_NUM_15) // ignored, ESP8266 UART1 will use fixed GPIO_2 
#define RXD_PIN (GPIO_NUM_4) 	// RX pin from Motor unit

#endif

/*
 Currently Neopixels are unavailable on ESP8266 due to timing problems (interrupts during WiFi are messing with bit banging 
 even though attempts to disable them is done). For some reason, this happens even when all the workarounds mentioned in 
 https://github.com/espressif/ESP8266_RTOS_SDK/issues/680 are tried.
*/
//#define NODE_USES_NEOPIXEL

void interface_init();
void handle_single_btn_click( int button );
void handle_long_btn_press( int button );
void handle_long_btn_release( int button );

void node_handle_led_set( uint8_t index, uint8_t r, uint8_t g, uint8_t b);

