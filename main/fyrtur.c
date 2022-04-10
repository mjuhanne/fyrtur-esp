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
#include "mqtt_manager.h"

#ifdef ESP32
#include "stm32ota.h"
#include "console.h"
#endif
#include "si7021.h"

#ifdef ESP32
#define TAG "fyrtur-esp32"
#else
#define TAG "fyrtur-esp8266"
#endif

#define NODE_BUILD_VERSION __DATE__ "-" __TIME__

#define DEFAULT_SENSOR_BROADCAST_INTERVAL 10 // seconds
#define NORMAL_MOTOR_SPEED 25  // rpm

#define MQTT_SETTING_ORIENTATION "orientation"
#define MQTT_SETTING_TARGET_POSITION "target_position"
#define MQTT_SETTING_LOCATION "location"
#define MQTT_SETTING_FULL_LEN "full_len"
#define MQTT_SETTING_MAX_LEN "max_len"
#define MQTT_SETTING_MOTOR_SPEED "speed"
#define MQTT_SETTING_DEFAULT_MOTOR_SPEED "default_speed"
#define MQTT_SETTING_MINIMUM_OPERATING_VOLTAGE "minimum_voltage"
#define MQTT_SETTING_DIAGNOSTICS "diagnostics"
#define MQTT_SETTING_SENSOR_BROADCAST_INTERVAL "sensor_interval"
#define MQTT_SETTING_CUSTOM_BUTTON_TOPIC "button_topic"

// Template for configuration topic to be used with Home Assistant's MQTT discovery.
// device_type: cover
// device_class: blind
#define BLIND_HA_CFG "{\
    \"name\": \"%s\", \
    \"unique_id\": \"%s\", \
    \"device_class\": \"blind\", \
    \"command_topic\": \"/home/control/%s/command\", \
    \"position_topic\": \"/home/cover/%s/position\", \
    \"set_position_topic\": \"/home/control/%s/set/target_position\", \
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

const char node_base_name[] = TAG;

static bool sensor_detected = false;
static uint32_t sensor_broadcast_interval;
static bool diagnostics = false;    // if this is set, more verbose diagnostics data is sent via MQTT
static bool publish_variables = true;

typedef enum {
    STM32_OTA_TASK_NONE = 0,
    STM32_OTA_TASK_FLASH,
    STM32_OTA_TASK_TEST_SW, // bootloader entry test with software hw_method
    STM32_OTA_TASK_TEST_HW, // bootloader entry test with hardware method (using RESET/BOOT0 pins). Just for debugging purposes
} stm32_ota_task_t;

stm32_ota_task_t stm32_ota_task = STM32_OTA_TASK_NONE;

char * custom_button_topic = NULL;

char * stm32_update_url = NULL;
char * stm32_update_md5 = NULL;

int node_handle_mqtt_set(void * arg) {
    iot_set_variable_return_code_t ret = IOT_OK;
    iot_variable_t * var = (iot_variable_t*)arg;
    if (strcmp(var->name,MQTT_SETTING_TARGET_POSITION)==0) {
        // Motor reports position between 0 (open) and 100 (closed). 
        // For Home Assistant this numbering is reversed and multiplied (0 = closed, 1000 = open )
        uint16_t pos = atoi(var->data);
        if (pos <= 1000) {
            blinds_go_to((float)(1000-pos)/10, false);
        } else {
            ESP_LOGE(TAG,"Invalid target position %d!", pos);
            mqtt_publish_error("Invalid target position!");
            ret = IOT_INVALID_VALUE; 
        }
    } else if (strcmp(var->name,MQTT_SETTING_LOCATION)==0) {
        uint16_t loc = atoi(var->data);
        if (loc <= 8192) {
            blinds_set_location(loc);
        } else {
            ESP_LOGE(TAG,"Invalid location %d!", loc);
            mqtt_publish_error("Invalid location!");
            ret = IOT_INVALID_VALUE; 
        }
    } else if (strcmp(var->name,MQTT_SETTING_MAX_LEN)==0) {
        blinds_set_max_length();
        if (diagnostics) {
            blinds_read_status_reg(EXT_LIMIT_STATUS_REG);   // read back the limits
        }
    } else if (strcmp(var->name,MQTT_SETTING_FULL_LEN)==0) {
        blinds_set_full_length();        
        if (diagnostics) {
            blinds_read_status_reg(EXT_LIMIT_STATUS_REG);   // read back the limits
        }
    } else if (strcmp(var->name,MQTT_SETTING_MOTOR_SPEED)==0) {
        int speed = atoi(var->data);
        if ( (speed >= 2) && (speed <= 25) ) {
            ESP_LOGI(TAG,"Setting motor speed to %d", speed);
            blinds_set_speed(speed);    // this is a temporary setting and will not be saved
        } else {
            ESP_LOGE(TAG,"Invalid speed %d!", speed);
            mqtt_publish_error("Invalid motor speed!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_DEFAULT_MOTOR_SPEED)==0) {
        int speed = atoi(var->data);
        if ( (speed >= 2) && (speed <= 25) ) {
            ESP_LOGI(TAG,"Setting default motor speed to %d", speed);
            blinds_set_default_speed(speed);    // value will be saved by motor module
        } else {
            ESP_LOGE(TAG,"Invalid speed %d!",speed);
            mqtt_publish_error("Invalid default motor speed!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_MINIMUM_OPERATING_VOLTAGE)==0) {
        float voltage = atof(var->data);
        if (voltage <= 8) {
            ESP_LOGI(TAG,"Setting minimum operating voltage to %.1f", voltage);
            blinds_set_minimum_voltage(voltage);    // value will be saved by motor module
        } else {
            ESP_LOGE(TAG,"Invalid minimum voltage %.1f!", voltage);
            mqtt_publish_error("Invalid minimum voltage!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_SENSOR_BROADCAST_INTERVAL)==0) {
        int interval = atoi(var->data);
        if (interval >= 0) {
            sensor_broadcast_interval = interval * 1000; // interval is stored as milliseconds
            ret = IOT_SAVE_VARIABLE;
        } else {
            ESP_LOGE(TAG,"Invalid broadcast interval %d!", interval);
            mqtt_publish_error("Invalid broadcast interval!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_DIAGNOSTICS)==0) {
        int dg = atoi(var->data);
        if (dg < 2) {
            diagnostics = dg;
            blinds_set_diagnostics(dg);
            ret = IOT_SAVE_VARIABLE;
        } else {
            ESP_LOGE(TAG,"Invalid diagnostics setting %d!", dg);
            mqtt_publish_error("Invalid diagnostics setting!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_ORIENTATION)==0) {
        blinds_orientation_t orientation = atoi(var->data);
        if (orientation < 2) {
            blinds_set_orientation(orientation);
        } else {
            ESP_LOGE(TAG,"Invalid orientation setting %d!", orientation);
            mqtt_publish_error("Invalid orientation setting!");
            ret = IOT_INVALID_VALUE;
        }
    } else if (strcmp(var->name,MQTT_SETTING_CUSTOM_BUTTON_TOPIC)==0) {
        if (strlen(var->data) > 1) {
            if (custom_button_topic != NULL) {
                free(custom_button_topic);
                custom_button_topic = NULL;
            }
            custom_button_topic = malloc(strlen(var->data)+1);
            strcpy(custom_button_topic, var->data);
            mqtt_manager_subscribe(custom_button_topic);
            ret = IOT_SAVE_VARIABLE;
        } else {
            ESP_LOGE(TAG,"Invalid custom button topic %s!", var->data);
            mqtt_publish_error("Invalid custom button topic!");
            ret = IOT_INVALID_VALUE;
        }
    } else {
        ret = IOT_VARIABLE_NOT_FOUND;
    }

    if ( (ret == IOT_VARIABLE_NOT_FOUND) || (ret == IOT_INVALID_VALUE) ) {
        IOT_MQTT_MSG_ERROR_LED();
    } else {
        IOT_MQTT_MSG_OK_LED();
    }


    return ret;
}


// Handle raw MQTT topic before any node-framework parsing (device_type, node_name etc)
int node_handle_generic_mqtt_msg(void * arg) {
    iot_mqtt_msg_t * msg = (iot_mqtt_msg_t*)arg;
    if (custom_button_topic) {
        if (strcmp(msg->arg, custom_button_topic)==0) {
            // we received button push topic
            if (strstr(msg->data,"open") != NULL) {
                handle_single_btn_click(0);
            } else if (strstr(msg->data,"close") != NULL) {
                handle_single_btn_click(1);
            } else if (strstr(msg->data,"stop") != NULL) {
                // we don't receive "button held down" event from remotes but instead a "stop" event, so regard it as so
                blinds_stop();
            }
        }
    }
    return 0;
}


int node_handle_mqtt_msg(void * arg) {
    iot_mqtt_msg_t * msg = (iot_mqtt_msg_t*)arg;
    int err = 0;
    if (strcmp(msg->device_type,"control")==0) {
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
                    err = 1;
            	}
            } else {
                ESP_LOGE(TAG,"command: no data!");
                err = 1;
            }
        } else if (strcmp(msg->subtopic,"reset_max_length")==0) {
            blinds_reset_max_length();
        } else if (strcmp(msg->subtopic,"force_move_up")==0) {
            if (msg->data) {
                float revs = atof(msg->data);
                blinds_move(DIRECTION_UP, revs, false, true);
            } else {
                ESP_LOGE(TAG,"force_move_up: number of revolutions not defined!");
                mqtt_publish_error("Number of revolutions not defined!");
                err = 1;
            }
        } else if (strcmp(msg->subtopic,"force_move_down")==0) {
            if (msg->data) {
                float revs = atof(msg->data);
                blinds_move(DIRECTION_DOWN, revs, false, true);
            } else {
                ESP_LOGE(TAG,"force_move_down: number of revolutions not defined!");
                mqtt_publish_error("Number of revolutions not defined!");
                err = 1;
            }
        } else if (strcmp(msg->subtopic,"stm32update")==0) {
            char * md5 = NULL;
            if ( (msg->data) && (md5=strchr(msg->data,' ')) ) {
                *md5 = 0;   // break msg->data into two strings
                md5++;
                if (stm32_update_url) {
                    free(stm32_update_url);
                    stm32_update_url = NULL;
                }
                if (stm32_update_md5) {
                    free(stm32_update_md5);
                    stm32_update_md5 = NULL;
                }
                stm32_update_url = malloc( strlen(msg->data)+ 1);
                if (stm32_update_url) {
                    strcpy(stm32_update_url, msg->data);
                    if (strcmp(md5,"0") != 0) {
                        stm32_update_md5 = malloc( strlen(md5) + 1);
                        if (stm32_update_md5) {
                            strcpy(stm32_update_md5, md5);
                        } else {
                            err = 1;
                        }
                    } else {
                        ESP_LOGW(TAG,"stm32update: MD5 check omitted");
                    }
                } else {
                    err = 1;                    
                }

                if (err != 1) {
                    // defer firmware update to the main process
                    stm32_ota_task = STM32_OTA_TASK_FLASH;
                } else {
                    ESP_LOGE(TAG,"stm32update: out of mem!");
                    mqtt_publish_error("stm32update: out of mem!");
                }

            } else {
                ESP_LOGE(TAG,"stm32update: both URL and md5 are needed!");
                mqtt_publish_error("stm32update: both URL and md5 are needed!");
                err = 1;
            }
        } else if (strcmp(msg->subtopic,"stm32info")==0) {
            int hw_method = 0;
            if (msg->data) {
                hw_method = atoi(msg->data);
            }
            if (hw_method) {
                stm32_ota_task = STM32_OTA_TASK_TEST_HW;
            } else {
                stm32_ota_task = STM32_OTA_TASK_TEST_SW;
            }
        } else if (strcmp(msg->subtopic,"toggle_orientation")==0) {
                blinds_toggle_orientation();
        } else if (strcmp(msg->subtopic,"reset_orientation")==0) {
                blinds_reset_orientation();
        } else if (strcmp(msg->subtopic,"button")==0) {
            if (msg->arg) {
                int btn = atoi(msg->arg);
                if ( (btn == 1) || (btn == 2) ) {
                    if (msg->data) {
                        if (strcmp(msg->data,"pushed")==0) {
                            handle_single_btn_click(btn-1);
                        } else if (strcmp(msg->data,"held")==0) {
                            handle_long_btn_press(btn-1);
                        } else if (strcmp(msg->data,"released")==0) {
                            handle_long_btn_release(btn-1);
                        }
                    } else {
                        ESP_LOGE(TAG,"button: invalid data!");
                        err = 1;
                    }
                } else {
                    ESP_LOGE(TAG,"button: invalid button (%s)!", msg->arg);
                    err = 1;
                }
            } else {
                ESP_LOGE(TAG,"button: no button defined!");
                err = 1;
            }
        } else {
            ESP_LOGE(TAG,"Invalid subtopic: '%s'", msg->subtopic);
            mqtt_publish_error("Invalid subtopic!");
            err = 1;
        }
    } else {
        ESP_LOGE(TAG,"Invalid device_type: '%s'", msg->device_type);
        mqtt_publish_error("Invalid device_type!");
        err = 1;
    }

    if (!err) {
        IOT_MQTT_MSG_OK_LED();
    } else {
        IOT_MQTT_MSG_ERROR_LED();
    }
    return 0;
}


void fyrtur_toggle_publish_variables( bool setting ) {
    publish_variables = setting;
}

void blinds_variable_updated( blinds_variable_t variable ) {
    if (!publish_variables)
        return;

    switch (variable) {
        case BLINDS_POSITION: {
            // Motor reports position between 0 (open) and 100 (closed). 
            // For Home Assistant this numbering is reversed and multiplied (0 = closed, 1000 = open )
            mqtt_publish_int("cover","position", 1000- (blinds_get_position()*10) );
        }
        break;

        case BLINDS_VOLTAGE: {
            mqtt_publish_float("cover","voltage", blinds_get_voltage());
        }
        break;

        case BLINDS_MOTOR_CURRENT: {
            mqtt_publish_float("cover","current", blinds_get_current());
        }
        break;

        case BLINDS_LOCATION: {
            mqtt_publish_int("cover","location", blinds_get_location());
        }
        break;

        case BLINDS_TARGET_LOCATION: {
            mqtt_publish_int("cover","target_location", blinds_get_target_location());
        }
        break;

        case BLINDS_SPEED: {
            mqtt_publish_int("cover","speed", blinds_get_speed());
        }
        break;

        case BLINDS_CALIBRATING_STATUS: {
            mqtt_publish_int("cover","calibrating", blinds_get_calibration_status());
        }
        break;

        case BLINDS_ORIENTATION: {
            mqtt_publish_int("cover","orientation", blinds_get_orientation());
        }
        break;

        case BLINDS_MAX_LEN: {
            mqtt_publish_int("cover","max_length", blinds_get_max_length());
        }
        break;

        case BLINDS_FULL_LEN: {
            mqtt_publish_int("cover","full_length", blinds_get_full_length());
        }
        break;

        case BLINDS_MOTOR_STATUS: {
            mqtt_publish("cover","motor_status", blinds_get_motor_status_str());
        }
        break;

        case BLINDS_STATUS: {
            mqtt_publish("cover","status", blinds_get_status_str());
        }
        break;

        case BLINDS_DIRECTION: {
            mqtt_publish("cover","direction", blinds_get_direction_str());
        }
        break;

        case BLINDS_TARGET_POSITION: {
            mqtt_publish_int("cover","target_position", 1000 - blinds_get_target_position()*10);
        }
        break;

        default:
            break;
    }
}


void node_publish_ha_cfg() {
    // we must publish the config to Home Assistant.
	// Configuration is published as "/home/cover/[node_name]/config"
    mqtt_publish_ha_cfg("cover", "config", BLIND_HA_CFG, 5);

    if (sensor_detected) {
        mqtt_publish_ha_cfg("sensor", "temperature/config", SENSOR_TEMPERATURE_CFG, 2);
        mqtt_publish_ha_cfg("sensor", "humidity/config", SENSOR_HUMIDITY_CFG, 2);
    }
}

void node_publish_node_info() {
    mqtt_publish_ext("node", "version", NODE_BUILD_VERSION, true);
    mqtt_publish_ext("node", "blinds_engine_version", blinds_get_build_version(), true);

    if (diagnostics) {
        if (custom_button_topic != NULL) {
            mqtt_publish_ext("node", "custom_button_topic", custom_button_topic, true);
        } else {            
            mqtt_publish_ext("node", "custom_button_topic", "Not defined", true);
        }
    }

    motor_firmware_status_t fw_status = blinds_get_firmware_status();
    if (fw_status == CUSTOM_FW) {
        mqtt_publish_ext("node", "motor_version", blinds_get_motor_version_str(), true);

        // calling blinds_variable_updated forces publishing these parameters
        blinds_variable_updated(BLINDS_POSITION);
        blinds_variable_updated(BLINDS_VOLTAGE);
        blinds_variable_updated(BLINDS_SPEED);
        blinds_variable_updated(BLINDS_ORIENTATION);
        blinds_variable_updated(BLINDS_MOTOR_STATUS);
        blinds_variable_updated(BLINDS_STATUS);
        blinds_variable_updated(BLINDS_DIRECTION);
        if (diagnostics) {
            blinds_variable_updated(BLINDS_LOCATION);
            blinds_variable_updated(BLINDS_TARGET_LOCATION);
            blinds_variable_updated(BLINDS_CALIBRATING_STATUS);
            blinds_variable_updated(BLINDS_MAX_LEN);
            blinds_variable_updated(BLINDS_FULL_LEN);
            blinds_variable_updated(BLINDS_MOTOR_CURRENT);
        }

    } else if (fw_status == ORIGINAL_FW) {
        mqtt_publish_ext("node", "motor_version", "Original", true);
        blinds_variable_updated(BLINDS_POSITION);
        blinds_variable_updated(BLINDS_VOLTAGE);
        blinds_variable_updated(BLINDS_SPEED);

    } else {
        mqtt_publish_ext("node", "motor_version", "Not detected!", true);        
    }
}

void node_handle_mqtt_connected() {
    ESP_LOGD(TAG, "node_handle_mqtt_connected - stack: %d", uxTaskGetStackHighWaterMark(NULL));
    node_publish_ha_cfg();
    node_publish_node_info();
    
    // subscribe to all fyrtur nodes
    mqtt_manager_subscribe("/home/control/fyrtur-blinds/#");

    // subscribe to custom Fyrtur remote topic
    if (custom_button_topic != NULL)
        mqtt_manager_subscribe(custom_button_topic);
}

int node_handle_name_change(void * arg) {
    node_publish_ha_cfg();
    node_publish_node_info();
    return 0;
}

int node_handle_pre_name_change(void * arg) {
    // Send empty configuration message to Home assistant in order to remove associations to old node name
    mqtt_publish_ha_cfg("cover", "config", NULL, 0);

    if (sensor_detected) {
        mqtt_publish_ha_cfg("sensor", "temperature/config", NULL, 0);
        mqtt_publish_ha_cfg("sensor", "humidity/config", NULL, 0);
    }
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

// only called after STM32 OTA
void node_handle_ota_success() {
    iot_led_set_priority(STATUS_LED, 0, 0, 0, -1); // release previously set priority lock
    IOT_LED_OFF();
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
    blinds_set_default_speed(NORMAL_MOTOR_SPEED);
    vTaskDelay( 50 / portTICK_PERIOD_MS );
    blinds_set_auto_cal(true);
    vTaskDelay( 50 / portTICK_PERIOD_MS );
    blinds_set_minimum_voltage(0);
    vTaskDelay( 50 / portTICK_PERIOD_MS );
    blinds_reset_full_length(); // Warning! This will NOT reset the full curtain length if using original firmware!
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
    start_console_task();
#endif

    ESP_LOGW(TAG, "Stack: %d", uxTaskGetStackHighWaterMark(NULL));

    // Initialize SI7021 / HTU21D temperature & humidity sensor
    if (si7021_init(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE) == SI7021_ERR_OK) {
        ESP_LOGI(TAG,"SI7021 / HTU21D temperature & humidity sensor detected. Sending periodic status updates..");
        sensor_detected = true;
    } else {
        ESP_LOGE(TAG,"Temperature sensor not found!");
    }

    // Initialize the blinds engine
    blinds_init();

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
    iot_set_callback( IOT_HANDLE_GENERIC_MQTT_MSG, node_handle_generic_mqtt_msg);
    iot_set_callback( IOT_HANDLE_SET_VARIABLE, node_handle_mqtt_set);
    iot_set_callback( IOT_HANDLE_CONN_STATUS, node_handle_conn_status);
    iot_set_callback( IOT_HANDLE_PRE_NAME_CHANGE, node_handle_pre_name_change);
    iot_set_callback( IOT_HANDLE_NAME_CHANGE, node_handle_name_change);
    iot_set_callback( IOT_HANDLE_FACTORY_RESET, node_handle_factory_reset);
    iot_set_callback( IOT_HANDLE_ERROR, node_handle_error);

    if (sensor_detected) {
        if (iot_get_nvs_uint32(MQTT_SETTING_SENSOR_BROADCAST_INTERVAL, &sensor_broadcast_interval)) {
            ESP_LOGI(TAG,"Setting sensor broadcast interval to %d seconds", sensor_broadcast_interval);
        } else {
            sensor_broadcast_interval = DEFAULT_SENSOR_BROADCAST_INTERVAL;
            ESP_LOGI(TAG,"Setting sensor broadcast interval to default setting of %d seconds", sensor_broadcast_interval);
        }
        sensor_broadcast_interval *= 1000;   // interval is stored as milliseconds
    }

    custom_button_topic = iot_get_nvs_str(MQTT_SETTING_CUSTOM_BUTTON_TOPIC);
    if (custom_button_topic) {
        ESP_LOGI(TAG,"Setting custom button topic to '%s'", custom_button_topic);
    } else {
        ESP_LOGW(TAG,"No custom button MQTT topic found");        
    }


    iot_get_nvs_bool(MQTT_SETTING_DIAGNOSTICS, &diagnostics);
    ESP_LOGI(TAG,"Setting verbose motor MQTT diagnostics to %d", diagnostics);
    blinds_set_diagnostics(diagnostics);

    if (blinds_get_firmware_status() == CUSTOM_FW) {    
        float speed;
        if (iot_get_nvs_float(MQTT_SETTING_MOTOR_SPEED, &speed)) {
            ESP_LOGI(TAG,"Setting motor speed to saved setting of %.1f rpm", speed);
        } else {
            speed = NORMAL_MOTOR_SPEED;
            ESP_LOGI(TAG,"Setting motor speed to normal setting of %.1f rpm", speed);
        }
        blinds_set_speed(speed);
    }    

#ifdef ESP32
    blinds_stm32_init(); // init STM32 OTA update engine
#endif

    // Console and temp & humidity sensor loop
    uint32_t sensor_timestamp = iot_timestamp();
    while (1) {

        if (sensor_detected) {
            if ( (sensor_broadcast_interval != 0) && (publish_variables) ) {   // broadcast only if it isn't disabled
                if (iot_timestamp() - sensor_timestamp > sensor_broadcast_interval ) {
                    sensor_timestamp = iot_timestamp();

                    float temperature = si7021_read_temperature();
                    float humidity = si7021_read_humidity();
                    
                    char temp_string[10];
                    char hum_string[10];
                    if (temperature == -999) 
                        strcpy(temp_string,"ERR");
                    else
                        snprintf(temp_string, 10,"%.1f", temperature);
                    if (humidity == -999) 
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

        if (stm32_ota_task != STM32_OTA_TASK_NONE) {
            node_handle_ota(NULL);

            // if testing, the url and md5 strings are both NULL
            if (blinds_stm32_ota(stm32_update_url, stm32_update_md5, (stm32_ota_task == STM32_OTA_TASK_TEST_HW))) {
                // read and publish new firmware version and motor unit status
                blinds_read_status_reg_blocking(EXT_STATUS_REG, 500);
                blinds_read_status_reg_blocking(EXT_VERSION_REG, 500);
                node_publish_node_info();
                node_handle_ota_success();                
            } else {
                node_handle_ota_failed();
            }
            stm32_ota_task = STM32_OTA_TASK_NONE;
            if (stm32_update_url) {
                free(stm32_update_url);
            }
            if (stm32_update_md5) {
                free(stm32_update_md5);
            }
            stm32_update_url = NULL;
            stm32_update_md5 = NULL;
        }
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
}
