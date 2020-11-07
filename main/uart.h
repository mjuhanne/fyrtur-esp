#include "driver/gpio.h"

int uart_write( const char * bytes, int bytes_num );
void init_uart();
int uart_read( uint8_t * rx_buffer, int bytes, int timeout );