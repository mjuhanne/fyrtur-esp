#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "uart.h"
#include "hw_interface.h"

#ifndef ESP32
#include "sw_serial.h"
SwSerial * swserial_handle = NULL;
#endif

static const char * TAG = "UART";

static const int UART_RX_BUF_SIZE = 1024;

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
#ifdef ESP32
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#else
    swserial_handle = sw_new( -1, RXD_PIN,  false, 256 );
    if (swserial_handle != NULL) {
        if (sw_open( swserial_handle, 2400 ) != ESP_OK) {
            ESP_LOGE(TAG,"Error initializing software serial on rx pin %d!", RXD_PIN);
        }
    }
#endif
}


int uart_write( const char * bytes, int bytes_num ) {
#ifdef ESP32
    return uart_write_bytes(UART_NUM_1, bytes, bytes_num);
#else
    return uart_write_bytes(UART_NUM_1, bytes, bytes_num);
#endif
}


int uart_read( uint8_t * rx_buffer, int bytes, int timeout ) {
#ifdef ESP32
    return uart_read_bytes(UART_NUM_1, rx_buffer, bytes, timeout);
#else
    if (swserial_handle == NULL)
        return 0;
    uint32_t now = esp_log_early_timestamp();
    int rx_bytes = 0;
    while ( (esp_log_early_timestamp() - now < timeout) && (rx_bytes < bytes) ) {
        int ch = sw_read(swserial_handle);
        if (ch != -1) {
            rx_buffer[rx_bytes] = ch;
            rx_bytes++;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);            
    }
    return rx_bytes;
#endif
}
