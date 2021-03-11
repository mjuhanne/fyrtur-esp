#include "stm_flash_http.h"

void blinds_stm32_init();

// hw_method = 1: use RESET/BOOT0 pins to enter bootloader. Just for debugging purposes
int blinds_stm32_ota( const char * url, const char * digest, uint8_t hw_method);
