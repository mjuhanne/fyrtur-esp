#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "../main/blinds.h"
#include "driver/gpio.h"
#include "si7021.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "../main/stm32ota.h"

static const char * TAG = "CONSOLE";

#define CONSOLE_TASK_CORE 1
#define PROMPT_STR CONFIG_IDF_TARGET

const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

extern void fyrtur_toggle_publish_variables( bool setting );


/** Arguments used by 'blinds' function */
static struct {
    struct arg_str *arg1;
    struct arg_str *arg2;
    struct arg_str *arg3;
    struct arg_end *end;
} blinds_cmd_args;

/** Arguments used by 'node' function */
static struct {
    struct arg_str *arg1;
    struct arg_str *arg2;
    struct arg_str *arg3;
    struct arg_str *arg4;
    struct arg_end *end;
} node_cmd_args;

static int console_blinds_cmd(int argc, char **argv)
{
    float revs, position, speed;
    int location;
    bool force_small_steps = false;
    unsigned int cmd_byte1, cmd_byte2;

    int nerrors = arg_parse(argc, argv, (void **) &blinds_cmd_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, blinds_cmd_args.end, argv[0]);
        return 1;
    }

    revs = 0;

    if ( (blinds_cmd_args.arg2->count > 0) && (strcmp(blinds_cmd_args.arg2->sval[0], "steps")==0) )
        force_small_steps = true;
    if ( (blinds_cmd_args.arg3->count > 0) && (strcmp(blinds_cmd_args.arg3->sval[0], "steps")==0) )
        force_small_steps = true;

    if (strcmp(blinds_cmd_args.arg1->sval[0], "up")==0) {
        if ( (blinds_cmd_args.arg2->count > 0) && (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) ) {
            revs = 0;
        }
        blinds_move(DIRECTION_UP, revs, force_small_steps, false);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "down")==0) {
        if ( (blinds_cmd_args.arg2->count > 0) && (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) ) {
            revs = 0;
        }
        blinds_move(DIRECTION_DOWN, revs, force_small_steps, false);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "goto")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &position) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (position)");
                return 1;
            } else {
                blinds_go_to(position, force_small_steps);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "goto_loc")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%d", &location) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (location)");
                return 1;
            } else {
                if (abs(location) < 8192) { // maximum parameter size is 12 bits. Lowermost bit is not transmitted so maximum is 8191
                    blinds_go_to_location(location);
                } else {
                    ESP_LOGE(TAG,"Location out of bounds!");
                }
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "speed")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &speed) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (speed)");
                return 1;
            } else {
                blinds_set_speed(speed);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "default_speed")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &speed) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (speed)");
                return 1;
            } else {
                blinds_set_default_speed(speed);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "minimum_voltage")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &position) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (voltage)");
                return 1;
            } else {
                blinds_set_minimum_voltage(position);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "stop")==0) {
        blinds_stop();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "status")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%d", &location) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (status register #)");
                return 1;
            }
            blinds_read_status_reg(location-1);
        } else {
            blinds_read_status_reg(STATUS_REG_1);
        }
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "location")==0) {
        blinds_read_status_reg(EXT_LOCATION_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_location")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%d", &location) != 1) {
                ESP_LOGE(TAG,"Invalid arg #1 (location)");
                return 1;
            } else {
                if (abs(location) < 8192) { // maximum parameter size is 12 bits. Lowermost bit is not transmitted so maximum is 8191
                    blinds_set_location(location);
                } else {
                    ESP_LOGE(TAG,"Location out of bounds!");
                }
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "version")==0) {
        blinds_read_status_reg(EXT_VERSION_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "debug")==0) {
        blinds_read_status_reg(EXT_DEBUG_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "sensor")==0) {
        blinds_read_status_reg(EXT_SENSOR_DEBUG_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "info")==0) {
        blinds_read_status_reg(EXT_STATUS_REG);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        blinds_read_status_reg(EXT_LOCATION_REG);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        blinds_read_status_reg(EXT_DEBUG_REG);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        blinds_read_status_reg(EXT_SENSOR_DEBUG_REG);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        blinds_read_status_reg(EXT_TUNING_PARAMS_REG);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        blinds_read_status_reg(EXT_VERSION_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "ext_status")==0) {
        blinds_read_status_reg(EXT_STATUS_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "limits")==0) {
        blinds_read_status_reg(EXT_LIMIT_STATUS_REG);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "reset")==0) {
        blinds_reset_max_length();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_max_len")==0) {
        blinds_set_max_length();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_full_len")==0) {
        blinds_set_full_length();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "force_up")==0) {
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) {
            ESP_LOGE(TAG,"Invalid arg #1 (revolutions)");
            return 1;
        }
        blinds_move(DIRECTION_UP, revs, force_small_steps, true);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "force_down")==0) {
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) {
            ESP_LOGE(TAG,"Invalid arg #1 (revolutions)");
            return 1;
        }
        blinds_move(DIRECTION_DOWN, revs, force_small_steps, true);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "raw")==0) {
        if ( (blinds_cmd_args.arg2->count == 0) || (blinds_cmd_args.arg3->count == 0) ) {
            ESP_LOGE(TAG,"No cmd_byte1 or cmd_byte2 argument given!");
            return 1;
        }
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%x", &cmd_byte1) != 1) {
            ESP_LOGE(TAG,"Invalid arg #1 (cmd_byte1)");
            return 1;
        }
        if (sscanf(blinds_cmd_args.arg3->sval[0], "%x", &cmd_byte2) != 1) {
            ESP_LOGE(TAG,"Invalid arg #2 (cmd_byte2)");
            return 1;
        }
        blinds_send_raw( cmd_byte1, cmd_byte2 );
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_auto_cal")==0) {
        if ( (sscanf(blinds_cmd_args.arg2->sval[0], "%d", &cmd_byte2) != 1) || (cmd_byte2 > 1) ) {
            ESP_LOGE(TAG,"Invalid arg #1 (auto_cal setting)");
            return 1;
        }
        blinds_set_auto_cal(cmd_byte2);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_orientation")==0) {
        if ( (sscanf(blinds_cmd_args.arg2->sval[0], "%d", &cmd_byte2) != 1) || (cmd_byte2 > 1) ) {
            ESP_LOGE(TAG,"Invalid arg #1 (orientation setting)");
            return 1;
        }
        blinds_set_orientation(cmd_byte2);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "stm32update")==0) {
        if (blinds_cmd_args.arg2->count == 1) {
            if (blinds_cmd_args.arg3->count == 1) {
                blinds_stm32_ota(blinds_cmd_args.arg2->sval[0], blinds_cmd_args.arg3->sval[0], 0 );
            } else {
                blinds_stm32_ota(blinds_cmd_args.arg2->sval[0], NULL, 0 );
            }
        } else {
            blinds_stm32_ota("http://firmware.local:80/fyrtur-stm32.bin", NULL, 0);
        } 
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "stm32info")==0) {
        if (blinds_cmd_args.arg2->count == 1) {
            blinds_stm32_ota(NULL, NULL, atoi(blinds_cmd_args.arg2->sval[0]));
        } else {
            blinds_stm32_ota(NULL, NULL, 0);
        }
    } else {
        ESP_LOGE(TAG, "Invalid arg %s", blinds_cmd_args.arg1->sval[0] );
    }
    return 0;
}


static int console_node_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &node_cmd_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, node_cmd_args.end, argv[0]);
        return 1;
    }

    if (strcmp(node_cmd_args.arg1->sval[0], "read_sensor")==0) {
        float temperature = si7021_read_temperature();
        float humidity = si7021_read_humidity();
        ESP_LOGI(TAG,"Temperature %f. Humidity %f", temperature, humidity);
    } else if (strcmp(node_cmd_args.arg1->sval[0], "start_ap")==0) {
        wifi_manager_send_message(WM_ORDER_START_AP, NULL);
    } else if (strcmp(node_cmd_args.arg1->sval[0], "stop_ap")==0) {
        wifi_manager_send_message(WM_ORDER_STOP_AP, NULL);
    } else if (strcmp(node_cmd_args.arg1->sval[0], "connect")==0) {
        if (argc == 4) {
            wifi_config_t* config = wifi_manager_get_wifi_sta_config();
            memset(config, 0x00, sizeof(wifi_config_t));
            memcpy(config->sta.ssid, node_cmd_args.arg2->sval[0], strlen(node_cmd_args.arg2->sval[0])+1);
            memcpy(config->sta.password, node_cmd_args.arg3->sval[0], strlen(node_cmd_args.arg3->sval[0])+1);
            ESP_LOGI(TAG, "ssid: %s, password: %s", node_cmd_args.arg2->sval[0], node_cmd_args.arg3->sval[0]);
            wifi_manager_connect_async();
        } else {
            ESP_LOGE(TAG,"Invalid number of args(%d)", argc);
        }
    } else if (strcmp(node_cmd_args.arg1->sval[0], "disconnect")==0) {
        wifi_manager_disconnect_async();
    } else if (strcmp(node_cmd_args.arg1->sval[0], "mqtt")==0) {
        if (argc > 1) {
            mqtt_manager_set_uri( node_cmd_args.arg2->sval[0] );
            if (argc > 3) {
                mqtt_manager_set_username( node_cmd_args.arg3->sval[0] );
                mqtt_manager_set_password( node_cmd_args.arg4->sval[0] );
            }
            mqtt_manager_connect_async();
        } else {
            ESP_LOGE(TAG,"Invalid number of args(%d)", argc);
        }
    } else if (strcmp(node_cmd_args.arg1->sval[0], "publish")==0) {
        if (argc > 1) {
            fyrtur_toggle_publish_variables( (strcmp(node_cmd_args.arg2->sval[0], "on") == 0) ? 1 : 0 );
        } else {
            ESP_LOGE(TAG,"Invalid number of args(%d)", argc);            
        }
    } else {
        ESP_LOGE(TAG,"Invalid command (%s)", node_cmd_args.arg1->sval[0]);        
    }
    return 0;
}




void initialize_console()
{
    ESP_LOGI(TAG,"Initializing console access...");

    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    // Disable buffering on stdin
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);


    // Configure UART. Note that REF_TICK is used so that the baud rate remains
    // correct while APB frequency is changing in light sleep mode.
    uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .source_clk = UART_SCLK_REF_TICK,            
    };
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    // Install UART driver for interrupt-driven reads and writes 
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );

    // Tell VFS to use UART driver
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    // Initialize the console
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    // Configure linenoise line completion library
    // Enable multiline editing. If not set, long commands will scroll within
    // single line.
    linenoiseSetMultiLine(1);

    // Tell linenoise where to get command completions and hints
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    // Set command history size
    linenoiseHistorySetMaxLen(100);

    esp_console_register_help_command();


    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = PROMPT_STR "> ";
#endif //CONFIG_LOG_COLORS
    }

    blinds_cmd_args.arg1 = arg_str1(NULL, NULL, "<arg1>", "command");
    blinds_cmd_args.arg2 = arg_str0(NULL, NULL, "<arg2>", "ARG #1 of command");
    blinds_cmd_args.arg3 = arg_str0(NULL, NULL, "<arg3>", "ARG #2 of command");
    blinds_cmd_args.end = arg_end(0);

    const esp_console_cmd_t blinds_cmd = {
        .command = "blinds",
        .help = "Control Fyrtur blinds",
        .hint = NULL,
        .func = &console_blinds_cmd,
        .argtable = &blinds_cmd_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&blinds_cmd) );

    node_cmd_args.arg1 = arg_str1(NULL, NULL, "<arg1>", "command");
    node_cmd_args.arg2 = arg_str0(NULL, NULL, "<arg2>", "ARG #1 of command");
    node_cmd_args.arg3 = arg_str0(NULL, NULL, "<arg3>", "ARG #2 of command");
    node_cmd_args.arg4 = arg_str0(NULL, NULL, "<arg4>", "ARG #3 of command");
    node_cmd_args.end = arg_end(0);

    // Maintenance commands for node. 
    const esp_console_cmd_t node_cmd = {
        .command = "node",
        .help = "Node maintenance functions",
        .hint = NULL,
        .func = &console_node_cmd,
        .argtable = &node_cmd_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&node_cmd) );

}

void console_task() {

    initialize_console();

    while (1) {

        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);

        if (line != NULL) {

            /* Add the command to the history if not empty*/
            if (strlen(line) > 0) {
                linenoiseHistoryAdd(line);
            }

            /* Try to run the command */
            int ret;
            esp_err_t err = esp_console_run(line, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unrecognized command\n");
            } else if (err == ESP_ERR_INVALID_ARG) {
                // command was empty
            } else if (err == ESP_OK && ret != ESP_OK) {
                printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
            } else if (err != ESP_OK) {
                printf("Internal error: %s\n", esp_err_to_name(err));
            }
            /* linenoise allocates line buffer on the heap, so need to free it */
            linenoiseFree(line);
        }

    }
}


void start_console_task() {
    xTaskCreatePinnedToCore(&console_task, "console_task", 4096, NULL, 5, NULL, CONSOLE_TASK_CORE);
}

