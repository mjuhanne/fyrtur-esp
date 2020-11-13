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

static const char * TAG = "CONSOLE";

#define PROMPT_STR CONFIG_IDF_TARGET

const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;


/** Arguments used by 'blinds' function */
static struct {
    struct arg_str *arg1;
    struct arg_str *arg2;
    struct arg_str *arg3;
    struct arg_end *end;
} blinds_cmd_args;


static int send_blinds_cmd(int argc, char **argv)
{
    static const char *SEND_CMD_TAG = "BLINDS_CMD";
    float revs, position, speed;
    bool silent=false;

    int nerrors = arg_parse(argc, argv, (void **) &blinds_cmd_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, blinds_cmd_args.end, argv[0]);
        return 1;
    }

    if ( (blinds_cmd_args.arg2->count > 0) && (strcmp(blinds_cmd_args.arg2->sval[0], "silent")==0) )
        silent=true;
    if ( (blinds_cmd_args.arg3->count > 0) && (strcmp(blinds_cmd_args.arg3->sval[0], "silent")==0) )
        silent=true;

    if (strcmp(blinds_cmd_args.arg1->sval[0], "up")==0) {
        if ( (blinds_cmd_args.arg2->count > 0) && (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) ) {
            revs=0;
        }
        blinds_move(DIRECTION_UP, revs, silent, false);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "down")==0) {
        if ( (blinds_cmd_args.arg2->count > 0) && (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) ) {
            revs=0;
        }
        blinds_move(DIRECTION_DOWN, revs, silent, false);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "goto")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &position) != 1) {
                ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (position)");
                return 1;
            } else {
                blinds_go_to(position, silent);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "speed")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &speed) != 1) {
                ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (speed)");
                return 1;
            } else {
                blinds_set_speed(speed);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "default_speed")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &speed) != 1) {
                ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (speed)");
                return 1;
            } else {
                blinds_set_default_speed(speed);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "minimum_voltage")==0) {
        if (blinds_cmd_args.arg2->count > 0) {
            if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &position) != 1) {
                ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (voltage)");
                return 1;
            } else {
                blinds_set_minimum_voltage(position);
            }
        }       
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "stop")==0) {
        blinds_stop();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "status")==0) {
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &position) != 1) {
            blinds_get_status(0);
        } else
            blinds_read_status_reg(position-1);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "limits")==0) {
        blinds_read_status_reg(4);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "reset")==0) {
        blinds_reset();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_max_len")==0) {
        blinds_set_max_length();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "set_full_len")==0) {
        blinds_set_full_length();
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "force_up")==0) {
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) {
            ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (revolutions)");
            return 1;
        }
        blinds_move(DIRECTION_UP, revs, silent, true);
    } else if (strcmp(blinds_cmd_args.arg1->sval[0], "force_down")==0) {
        if (sscanf(blinds_cmd_args.arg2->sval[0], "%f", &revs) != 1) {
            ESP_LOGE(SEND_CMD_TAG,"Invalid arg #2 (revolutions)");
            return 1;
        }
        blinds_move(DIRECTION_DOWN, revs, silent, true);
    } else {
        ESP_LOGE(TAG, "Invalid arg %s", blinds_cmd_args.arg1->sval[0] );
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

    blinds_cmd_args.arg1 = arg_str1(NULL, NULL, "<arg1>", "ARG #1 of command");
    blinds_cmd_args.arg2 = arg_str0(NULL, NULL, "<arg2>", "ARG #2 of command");
    blinds_cmd_args.arg3 = arg_str0(NULL, NULL, "<arg3>", "ARG #3 of command");
    blinds_cmd_args.end = arg_end(2);

    const esp_console_cmd_t blinds_cmd = {
        .command = "blinds",
        .help = "Send command to blinds",
        .hint = NULL,
        .func = &send_blinds_cmd,
        .argtable = &blinds_cmd_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&blinds_cmd) );
}

bool run_console() {

    /* Get a line using linenoise.
     * The line is returned when ENTER is pressed.
     */
    char* line = linenoise(prompt);
    if (line == NULL) { /* Break on EOF or error */
        return false;
    }
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
    return true;
}
