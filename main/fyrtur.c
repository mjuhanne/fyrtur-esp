/* 
    MQTT Fyrtur controller for ESP32 and ESP8266
    (c) Marko Juhanne 2020

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "node-framework.h"
#include "hw_interface.h"
#include "blinds.h"
#ifdef ESP32
extern void initialize_console();
extern bool run_console();
#endif
#include "si7021.h"


#define TAG "fyrtur"
#define DEFAULT_SENSOR_BROADCAST_INTERVAL 10 // seconds
#define DEFAULT_MOTOR_SPEED 25  // rpm

#define MQTT_SETTING_MOTOR_SPEED "speed"
#define MQTT_SETTING_DEFAULT_MOTOR_SPEED "default_speed"
#define MQTT_SETTING_MINIMUM_OPERATING_VOLTAGE "minimum_voltage"

#define MQTT_SETTING_SENSOR_BROADCAST_INTERVAL "sensor_broadcast_interval"

const char node_base_name[] = TAG;

#define NODE_BUILD_VERSION __DATE__ "-" __TIME__

static bool sensor_detected = false;
static uint32_t sensor_broadcast_interval = DEFAULT_SENSOR_BROADCAST_INTERVAL * 1000;   // interval is stored as milliseconds

int node_handle_mqtt_set(void * arg) {
    iot_variable_t * var = (iot_variable_t*)arg;
    if (strcmp(var->name,MQTT_SETTING_MOTOR_SPEED)==0) {
        float speed = atof(var->data);
        if ( (speed >= 2) && (speed <= 25) ) {
            ESP_LOGI(TAG,"Setting motor speed to %.1f", speed);
            blinds_set_speed(speed);
        } else {
            ESP_LOGE(TAG,"Invalid speed %.1f!",speed);
            mqtt_publish_error("Invalid motor speed!");
        }
        return 1;
    } else if (strcmp(var->name,MQTT_SETTING_DEFAULT_MOTOR_SPEED)==0) {
        float speed = atof(var->data);
        if ( (speed >= 2) && (speed <= 25) ) {
            ESP_LOGI(TAG,"Setting default motor speed to %.1f", speed);
            blinds_set_default_speed(speed);
        } else {
            ESP_LOGE(TAG,"Invalid speed %.1f!",speed);
            mqtt_publish_error("Invalid default motor speed!");
        }
        return 1;
    } else if (strcmp(var->name,MQTT_SETTING_MINIMUM_OPERATING_VOLTAGE)==0) {
        float voltage = atof(var->data);
        if (voltage <= 8) {
            ESP_LOGI(TAG,"Setting minimum operating voltage to %.1f", voltage);
            blinds_set_minimum_voltage(voltage);
        } else {
            ESP_LOGE(TAG,"Invalid minimum voltage %.1f!",voltage);
            mqtt_publish_error("Invalid minimum voltage!");
        }
        return 1;
    } else if (strcmp(var->name,MQTT_SETTING_SENSOR_BROADCAST_INTERVAL)==0) {
        int interval = atoi(var->data);
        if (interval >= 0) {
            sensor_broadcast_interval = interval * 1000; // interval is stored as milliseconds
        } else {
            ESP_LOGE(TAG,"Invalid broadcast interval %d!",interval);
            mqtt_publish_error("Invalid interval!");
        }
        return 1;
    }
    return 0;
}

int node_handle_mqtt_msg(void * arg) {
    iot_mqtt_msg_t * msg = (iot_mqtt_msg_t*)arg;
    if (strcmp(msg->device_type,"cover")==0) {
        if (strcmp(msg->subtopic,"command")==0) {
            if (msg->data) {
            	if (strcmp(msg->data,"OPEN")==0)
            		blinds_move(DIRECTION_UP, 0, false, false);
            	else if (strcmp(msg->data,"CLOSE")==0)
            		blinds_move(DIRECTION_DOWN, 0, false, false);
                else if (strcmp(msg->data,"STOP")==0)
                    blinds_stop();
            	else {
                    ESP_LOGE(TAG,"command: invalid cmd!");
            	}
            } else {
                ESP_LOGE(TAG,"command: no data!");
            }
        } else if (strcmp(msg->subtopic,"set_position")==0) {
            if (msg->data) {
                // Motor reports position between 0 (open) and 100 (closed). 
                // For Home Assistant this numbering is reversed and multiplied (0 = closed, 1000 = open )
            	uint16_t pos = atoi(msg->data);
            	blinds_go_to((float)(1000-pos)/10, false);
            } else {
                ESP_LOGE(TAG,"set_position: no pos!");
            }
        } else if (strcmp(msg->subtopic,"reset")==0) {
            blinds_reset();
        } else if (strcmp(msg->subtopic,"set_max_len")==0) {
            blinds_set_max_length();
        } else if (strcmp(msg->subtopic,"set_full_len")==0) {
            blinds_set_full_length();
        } else if (strcmp(msg->subtopic,"force_move_up")==0) {
            if (msg->data) {
                float revs = atof(msg->data);
                blinds_move(DIRECTION_UP, revs, false, true);
            } else {
                ESP_LOGE(TAG,"force_move_up: number of revolutions not defined!");
                mqtt_publish_error("Number of revolutions not defined!");
            }
        } else if (strcmp(msg->subtopic,"force_move_down")==0) {
            if (msg->data) {
                float revs = atof(msg->data);
                blinds_move(DIRECTION_DOWN, revs, false, true);
            } else {
                ESP_LOGE(TAG,"force_move_down: number of revolutions not defined!");
                mqtt_publish_error("Number of revolutions not defined!");
            }
        } else if (strcmp(msg->subtopic,"position")==0) {
        	// ignore position status sent by us
        } else {
            ESP_LOGE(TAG,"Invalid subtopic: '%s'", msg->subtopic);
            mqtt_publish_error("Invalid subtopic!");
        }
    } else if (strcmp(msg->device_type,"sensor")==0) {
        // ignore sensor updates sent by us
    } else {
        ESP_LOGE(TAG,"Invalid device_type: '%s'", msg->device_type);
        mqtt_publish_error("Invalid device_type!");
    }
    return 0;
}

void blinds_motor_position_updated( float position ) {
    // Motor reports position between 0 (open) and 100 (closed). 
    // For Home Assistant this numbering is reversed and multiplied (0 = closed, 1000 = open )
	mqtt_publish_int("cover","position",1000-position*10);
}

// Template for configuration topic to be used with Home Assistant's MQTT discovery.
// device_type: cover
// device_class: blind
#define BLIND_HA_CFG "{\
    \"name\": \"%s\", \
    \"device_class\": \"blind\", \
    \"command_topic\": \"/home/cover/%s/command\", \
    \"position_topic\": \"/home/cover/%s/position\", \
    \"set_position_topic\": \"/home/cover/%s/set_position\", \
    \"position_open\": 1000 \
    }"


// Optional temperature and humidity sensor template for configuration topic to be used with Home Assistant's MQTT discovery.
// device_type: sensor
// device_class: temperature
#define SENSOR_TEMPERATURE_CFG "{\
    \"name\": \"%s temperature\", \
    \"device_class\": \"temperature\", \
    \"state_topic\": \"/home/sensor/%s/temperature\" \
    }"

// device_type: sensor
// device_class: humidity
#define SENSOR_HUMIDITY_CFG "{\
    \"name\": \"%s humidity\", \
    \"device_class\": \"humidity\", \
    \"state_topic\": \"/home/sensor/%s/humidity\" \
    }"


void node_publish_ha_cfg() {
    // we must publish the config to Home Assistant.
	// Configuration is published as "/home/cover/[node_name]/config"
    mqtt_publish_ha_cfg("cover", "config", BLIND_HA_CFG, 4);

    if (sensor_detected) {
        mqtt_publish_ha_cfg("sensor", "temperature/config", SENSOR_TEMPERATURE_CFG, 2);
        mqtt_publish_ha_cfg("sensor", "humidity/config", SENSOR_HUMIDITY_CFG, 2);
    }

    // publish current curtain state
    blinds_motor_position_updated( blinds_get_pos() );

}

void node_handle_mqtt_connected() {
    ESP_LOGD(TAG, "node_handle_mqtt_connected Stack: %d", uxTaskGetStackHighWaterMark(NULL));

    node_publish_ha_cfg();
    mqtt_publish_ext("node", "version", NODE_BUILD_VERSION, true);

    if (blinds_is_custom_firmware()) {
        mqtt_publish_ext("node", "motor_version", blinds_get_version(), true);
    } else {
        mqtt_publish_ext("node", "motor_version", "Original", true);        
    }
}

int node_handle_name_change(void * arg) {
    node_publish_ha_cfg();
    return 0;
}


int node_handle_ota( void * arg ) {
    IOT_OTA_STARTING();
    return 0;
}

void node_handle_ota_failed() {
    iot_led_set_priority(STATUS_LED, 0, 0, 0, -1); // release previously set priority lock
    IOT_OTA_FAILED();
}


int node_handle_conn_status( void * arg) {
    iot_conn_status_t status = (iot_conn_status_t)arg;
    //ESP_LOGI(TAG, "node_handle_conn_status Stack: %d status %d", uxTaskGetStackHighWaterMark(NULL), status);

    switch (status) {
        case IOT_WIFI_DISCONNECTED: 
            IOT_WIFI_DISCONNECTED_LED();
            break;
        case IOT_WIFI_CONNECTING: 
            IOT_WIFI_CONNECTING_LED();
            break;
        case IOT_WIFI_CONNECTED:
            IOT_WIFI_CONNECTED_LED();
            break;
        case IOT_MQTT_CONNECTING:
            IOT_MQTT_CONNECTING_LED();
            break;
        case IOT_MQTT_CONNECTED:
            IOT_MQTT_CONNECTED_LED()
            node_handle_mqtt_connected();
            break;
        case IOT_MQTT_DISCONNECTED:
            IOT_MQTT_DISCONNECTED_LED();
            break;
        default:
            break;
    }

    return 0;
}


int node_handle_error( void * arg) {
    iot_error_code_t err = (iot_error_code_t)arg;
    if (err==IOT_OTA_ERROR) {
        node_handle_ota_failed();
    }
    return 0;
}

int node_handle_factory_reset( void * arg) {
    iot_led_pulse(STATUS_LED, 0, 0, 30, 1000, 1000, -1, NORMAL_LED_PRIORITY);
    // TODO
    return 0;
}




void app_main()
{
#ifdef ESP32
    ESP_LOGI(TAG, "MQTT Fyrtur controller for ESP32");
#else
    ESP_LOGI(TAG, "MQTT Fyrtur controller for ESP8266");
#endif
    // Initialize hardware interface (buttons and the LED)
    interface_init();

    // Signal powering on
    node_handle_led_set(STATUS_LED, IOT_BTN_LED_COLOR);

    ESP_LOGW(TAG, "Stack: %d heap: %d", uxTaskGetStackHighWaterMark(NULL), esp_get_free_heap_size());

#ifdef ESP32
    // Init console access. This is just for debugging purposes and used only on ESP32 since ESP8266 has scarce memory
    initialize_console();
#endif

    // Initialize SI7021 / HTU21D temperature & humidity sensor
    if (si7021_init(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE) == SI7021_ERR_OK) {
        ESP_LOGI(TAG,"SI7021 / HTU21D temperature & humidity sensor detected. Sending periodic status updates..");
        sensor_detected = true;
    } else {
        ESP_LOGE(TAG,"Temperature sensor not found!");
    }

    // Initialize the blinds engine
    blinds_init();

    // Set motor speed
    /*
    if (blinds_is_custom_firmware()) {    
        float speed;
        if (iot_get_nvs_float(MQTT_SETTING_MOTOR_SPEED, &speed)) {
            ESP_LOGI(TAG,"Setting motor speed to saved setting of %.1f rpm", speed);
        } else {
            speed = DEFAULT_MOTOR_SPEED;
            ESP_LOGI(TAG,"Setting motor speed to default setting of %.1f rpm", speed);
        }
        blinds_set_speed(speed);
    }    
    */

    // set verbose logging for debugging purposes
    iot_logging();

    // TODO: stack analyzing
    esp_log_level_set("wifi_manager", ESP_LOG_DEBUG);
    esp_log_level_set("mqtt_manager", ESP_LOG_DEBUG);
    esp_log_level_set("LED", ESP_LOG_DEBUG);
    esp_log_level_set("INTERFACE", ESP_LOG_DEBUG);

    // Initialize node framework
    iot_init(node_base_name);

    // Repeated blinking to signal unconfigured state
    IOT_UNCONFIGURED_LED();

    // Callbacks for node framework events
    iot_set_callback( IOT_HANDLE_OTA, node_handle_ota);
    iot_set_callback( IOT_HANDLE_MQTT_MSG, node_handle_mqtt_msg);
    iot_set_callback( IOT_HANDLE_SET_VARIABLE, node_handle_mqtt_set);
    iot_set_callback( IOT_HANDLE_CONN_STATUS, node_handle_conn_status);
    iot_set_callback( IOT_HANDLE_FACTORY_RESET, node_handle_factory_reset);
    iot_set_callback( IOT_HANDLE_ERROR, node_handle_error);

    // Console and temp & humidity sensor loop
    uint32_t sensor_timestamp = iot_timestamp();
    while (1) {

        if (sensor_detected) {
            if (sensor_broadcast_interval != 0) {   // broadcast only if it isn't disabled
                if (iot_timestamp() - sensor_timestamp > sensor_broadcast_interval ) {
                    sensor_timestamp = iot_timestamp();

                    float temperature = si7021_read_temperature();
                    float humidity = si7021_read_humidity();
                    
                    char temp_string[10];
                    char hum_string[10];
                    if (temperature==-999) 
                        strcpy(temp_string,"ERR");
                    else
                        snprintf(temp_string, 10,"%.1f", temperature);
                    if (humidity==-999) 
                        strcpy(hum_string,"ERR");
                    else
                        snprintf(hum_string, 10,"%.0f", humidity);
                    ESP_LOGI(TAG, "Sensor: %s°C - %s%%", temp_string, hum_string);
                    if (iot_is_connected()) {
                        mqtt_publish("sensor", "temperature", temp_string);
                        mqtt_publish("sensor", "humidity", hum_string);                
                    }
                }
            }
        }

#ifdef ESP32
        run_console();
#endif
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
}
