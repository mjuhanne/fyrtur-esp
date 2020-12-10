#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "iot_helper.h"

#include "hw_interface.h"
#include "node-framework.h"
#include "wifi_manager.h"
#include "blinds.h"
#ifdef NODE_USES_NEOPIXEL
#include "ws2812.h"
#endif

static const char * TAG = "INTERFACE";

TaskHandle_t sensor_task;

int saved_target_speed;

#define GPIO_SENSORS_PIN_SEL ( \
    (1ULL<<BTN1_GPIO) | \
    (1ULL<<BTN2_GPIO) | \
    (0) )

#define GPIO_OUTPUT_PIN_SEL (\
    (1ULL<<LED_GPIO) | \
    (0) )

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

unsigned long btn_timestamps[MAX_BUTTONS];
unsigned long last_short_click_timestamps[MAX_BUTTONS];
int btn_states[MAX_BUTTONS];

typedef enum {
    NO_DOUBLE_BTN_PRESS = 0,
    OTHER_BTN_RELEASED,
    NO_FUNCTION,
    START_AP,
    FACTORY_RESET_IMMINENT,
    FACTORY_RESET
} double_btn_stage_t;

double_btn_stage_t double_btn_stage = NO_FUNCTION;



static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void handle_single_btn_click( int button ) {
    ESP_LOGI(TAG, "stack: %d", uxTaskGetStackHighWaterMark(NULL));
    IOT_SHORT_CLICK_LED();

    if (blinds_get_status() == BLINDS_STOPPED) {
        if (button==BTN_1) {
            blinds_move(DIRECTION_UP, 0, false, false);
        } else if (button==BTN_2) {
            blinds_move(DIRECTION_DOWN, 0, false, false);
        }
    } else {
        // stop otherwise
        blinds_stop();
    }
}

void handle_long_btn_press( int button ) {

    if (blinds_get_status() == BLINDS_STOPPED) {
        if (blinds_get_firmware_status() == ORIGINAL_FW) {
            // unrestricted movement but with small steps
            if (button==BTN_1) {
                blinds_move(DIRECTION_UP, -1, true, true);
            } else if (button==BTN_2) {
                blinds_move(DIRECTION_DOWN, -1, true, true);
            }
        } else {
            // unrestricted movement but with lower speed
            saved_target_speed = blinds_get_target_speed();
            blinds_set_speed(SLOW_MOVEMENT_SPEED);
            if (button==BTN_1) {
                blinds_move(DIRECTION_UP, -1, false, true);
            } else if (button==BTN_2) {
                blinds_move(DIRECTION_DOWN, -1, false, true);
            }
        }
    } else {
        // stop otherwise
        blinds_stop();
    }
}

void handle_long_btn_release( int button ) {
    // stop movement
    blinds_stop();

    if (blinds_get_firmware_status() == CUSTOM_FW) {
        // restore the original speed
        blinds_set_speed(saved_target_speed);
    }
}


void check_single_click() {
    for (int i=0;i<MAX_BUTTONS;i++) {
        if (last_short_click_timestamps[i] != 0) {

            if ( (blinds_get_status() == BLINDS_MOVING) || (iot_timestamp() - last_short_click_timestamps[i] > DOUBLE_CLICK_PERIOD) ) {
                // We are currently moving and so we want to stop the curtains immediately -> don't wait for double-click
                // OR
                // Enough time has passed so that we don't expect a second click -> this has been a single click
                ESP_LOGI(TAG, "Single click (btn %d)", i+1);
                handle_single_btn_click(i);
                last_short_click_timestamps[i] = 0;
            }
        }
    }
}

void check_single_button_long_press() {
    if (double_btn_stage == NO_DOUBLE_BTN_PRESS) {        
        for (int i=0;i<MAX_BUTTONS;i++) {
            if (btn_timestamps[i] != 0) {
                if (iot_timestamp() - btn_timestamps[i] > LONG_PRESS_PERIOD) {
                    // this was a long press
                    ESP_LOGI(TAG, "Long button press (btn %d)", i+1);
                    handle_long_btn_press(i);
                    btn_timestamps[i] = 0;
                }
            }
        }
    }
}


bool check_double_click( int button ) {
    if (iot_timestamp() - last_short_click_timestamps[button] < DOUBLE_CLICK_PERIOD) {
        // this button has been double click
        ESP_LOGI(TAG, "Double click (btn %d)", button+1);

        if (blinds_get_position() != 0) {
            // double click of either of the buttons causes SET_SOFT_LOWER_LIMIT when curtains are below top position
            IOT_SET_LIMITS_LED();
            
            blinds_set_max_length();            
        } else {
            // double click while curtains at top position -> reset maximum length
            ESP_LOGW(TAG,"Reset maximum length and calibrate");
            
            blinds_reset_max_length();

            IOT_RESET_LIMITS_LED();
        }

        last_short_click_timestamps[button] = 0;
        return 1;
    }
    return 0;
}


/*
 * Here we handle only those two-button functions that require that button(s) have been released 
 * (in contrast to other kind of functions that can be invoked even if buttons are kept pressed )
 */

void check_double_btn_released() {
    if (double_btn_stage == FACTORY_RESET_IMMINENT) {
        // factory reset was canceled. stop the blinking led
        IOT_LED_OFF();
    }

    double_btn_stage = OTHER_BTN_RELEASED;
}

/*
 * Here we handle the two-button functions when both buttons are still kept pressed
 */
void check_double_btn_kept_pressed() {
    if (btn_states[BTN_1] && btn_states[BTN_2]) {
        if ( (iot_timestamp()-btn_timestamps[BTN_1] > FACTORY_RESET_THRESHOLD)
            && (iot_timestamp()-btn_timestamps[BTN_2] > FACTORY_RESET_THRESHOLD) ) {
            if (double_btn_stage != FACTORY_RESET) {
                IOT_FACTORY_RESET_LED();
                iot_factory_reset();
                double_btn_stage = FACTORY_RESET;
            }
        } else if ( (iot_timestamp()-btn_timestamps[BTN_1] > FACTORY_RESET_IMMINENT_THRESHOLD)
            && (iot_timestamp()-btn_timestamps[BTN_2] > FACTORY_RESET_IMMINENT_THRESHOLD) ) {
            if (double_btn_stage != FACTORY_RESET_IMMINENT) {
                ESP_LOGW(TAG,"Factory reset imminent!");
                double_btn_stage = FACTORY_RESET_IMMINENT;
                IOT_FACTORY_RESET_IMMINENT_LED();
            }
        } else if ( (iot_timestamp()-btn_timestamps[BTN_1] > START_AP_THRESHOLD)
            && (iot_timestamp()-btn_timestamps[BTN_2] > START_AP_THRESHOLD) ) {
            if (double_btn_stage != START_AP) {
                ESP_LOGW(TAG,"Start AP");
                wifi_manager_set_auto_ap_shutdown(false);
                wifi_manager_send_message(WM_ORDER_START_AP, NULL);
                double_btn_stage = START_AP;
                IOT_START_AP_LED();
            }
        } else {
            // both buttons are pressed but not yet long enough
            double_btn_stage = NO_FUNCTION;
        }
    } 

    if (!btn_states[BTN_1] && !btn_states[BTN_2]) {
        // both buttons have been released
        double_btn_stage = NO_DOUBLE_BTN_PRESS;
    }        
}


void handle_btn_int( int btn, int status) {
    if (status != btn_states[btn]) {
        ESP_LOGI(TAG,"BTN(%d) status changed: %d",btn+1, status);
        ESP_LOGI(TAG, "Stack: %d", uxTaskGetStackHighWaterMark(NULL));

        if (status) {
            // button pressed. Log the time this event
            btn_timestamps[btn] = iot_timestamp();
        }
        else {
            // a button is released. 

            if (double_btn_stage == NO_DOUBLE_BTN_PRESS) {
                // a button is released but the other hasn't been touched -> this has been a single button release

                if (btn_timestamps[btn] == 0) {
                    // a button was released but the timestamp has been cleared already. 
                    // This was a release of a single button long press.
                    handle_long_btn_release(btn);
                } else {                    
                    // this was a short click. Check if another single click was done previously (so it's a double click)
                    if (!check_double_click(btn)) {
                        // no double click yet. Save this timestamp and act upon this event a bit later when we know this will not have been double click
                        last_short_click_timestamps[btn] = iot_timestamp();
                    }
                }
            } else {

                // Check if both buttons had been held down and if that means we have to invoke a function
                check_double_btn_released();
            }

            // reset the timestamp
            btn_timestamps[btn] = 0;
        }
        btn_states[btn] = status;
    }
}


static void sensor_loop(void *arg)
{
    ESP_LOGI(TAG,"[SENSOR] Running on core #%d", xPortGetCoreID());

    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_RATE_MS)) {
            int status = gpio_get_level(io_num) ? 0 : 1;  // reverse logic
            if (io_num==BTN1_GPIO) {
                handle_btn_int( BTN_1, status );
            } else if (io_num==BTN2_GPIO) {
                handle_btn_int( BTN_2, status );
            } else {
                ESP_LOGE(TAG,"Unknown sensor pin: %d", io_num);
            }
        }
        check_single_button_long_press();
        check_double_btn_kept_pressed();
        check_single_click();
    }
}


void interface_init() {
    // ---- configure output pins
    gpio_config_t io_conf;
    //disable interrupt
#ifdef ESP32
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
#else
    io_conf.intr_type = GPIO_INTR_DISABLE;
#endif
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);    


    // --- configure sensor pins
    //gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_SENSORS_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start sensor task
#ifdef ESP32
    xTaskCreatePinnedToCore(&sensor_loop, "sensor_task", 4096, NULL, 10, &sensor_task, SENSOR_TASK_CORE);
#else
    xTaskCreatePinnedToCore(&sensor_loop, "sensor_task", 1024, NULL, 10, &sensor_task, SENSOR_TASK_CORE);
#endif

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BTN1_GPIO, gpio_isr_handler, (void*) BTN1_GPIO);
    gpio_isr_handler_add(BTN2_GPIO, gpio_isr_handler, (void*) BTN2_GPIO);

    for (int i=0;i<MAX_BUTTONS;i++) {
        btn_states[i] = 0;
        btn_timestamps[i] = 0;
        last_short_click_timestamps[i] = 0;
    }

}


void IRAM_ATTR node_handle_led_set( uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
#ifdef NODE_USES_NEOPIXEL
    ws2812_set(LED_GPIO, WS2812_RGB( r, g, b));
#else
    if (r>0 || g>0 || b>0) {
        gpio_set_level(LED_GPIO, 1); 
        //ESP_LOGI(TAG,"led on");
    }
    else {
        gpio_set_level(LED_GPIO, 0);
        //ESP_LOGI(TAG,"led off");
    }
#endif
}

