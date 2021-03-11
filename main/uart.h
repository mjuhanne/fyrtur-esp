#include "driver/gpio.h"

int uart_write( uint8_t * bytes, int bytes_num );
int uart_read( uint8_t * rx_buffer, int bytes, int timeout );

void uart_config();
void init_uart();
