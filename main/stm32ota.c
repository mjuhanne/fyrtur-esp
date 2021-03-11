#include "stm32ota.h"
#include "node-framework.h"
#include "hw_interface.h"
#include "blinds.h"
#include "uart.h"

#define STM32F030_FLASH_PAGE_SIZE 1024
#define STM32F030_BOOTLOADER_DELAY 500
#define STM32F030_MEMORY_ADDR 0x08000000
#define STM32_BOOTLOADER_BAUD_RATE 38400

extern SemaphoreHandle_t uart_semaphore;
extern blinds_motor_status_t blinds_motor_status;

static const char * TAG = "STM32OTA";

uint8_t enable_debug_msg;

void Flash_Log(const char* format, ...)
{
    char buf[80];
    va_list va;
    va_start(va, format);
    vsnprintf(buf, 80, format, va);
    va_end(va);
    printf("%s\r\n", buf);
    mqtt_publish("node", "stm32ota_log", buf);

}

void stm32_flash_callback( flash_task_t task, flash_stage_t stage, int block, int max_blocks, const char * msg ) {
    switch (stage) {
        case FLASH_STAGE_START: {
            Flash_Log("Starting Flash %s task", flash_task2txt(task));
        } break;

        case FLASH_STAGE_CONTINUE: {
            if (task == FLASH_DOWNLOAD) {
                Flash_Log("Downloading file: %d/%d", block, max_blocks);
            } else if (task == FLASH_DOWNLOAD_MD5) {
                Flash_Log("Downloading MD5 file: %d/%d", block, max_blocks);
            } else if (task == FLASH_ERASE) {
                Flash_Log("Erasing page %d/%d", block, max_blocks);
            } else if (task == FLASH_WRITE) {
                Flash_Log("Writing block %d/%d", block, max_blocks);
            } else if (task == FLASH_READ) {
                Flash_Log("Verifying block %d/%d", block, max_blocks);
            };
        } break;

        case FLASH_STAGE_COMPLETE: {
            Flash_Log("Flash %s task complete!", flash_task2txt(task));
        } break;

        case FLASH_STAGE_LOG_DEBUG: {
            if (enable_debug_msg) {
                if (msg) {
                    Flash_Log("Flash %s task: %s", flash_task2txt(task), msg);
                }                
            }
        } break;

        case FLASH_STAGE_LOG_ERROR: 
            // fall-through
        case FLASH_STAGE_LOG_INFO: {
            if (msg) {
                Flash_Log("Flash %s task: %s", flash_task2txt(task), msg);
            }
        } break;

        case FLASH_STAGE_ERROR: {
            if (msg) {
                Flash_Log("Flash %s task error! %s", flash_task2txt(task), msg);
            } else {
                Flash_Log("Flash %s task undefined error!", flash_task2txt(task));
            }
        } break;

        default: 
            break;
    }
}


int blinds_stm32_ota( const char * url, const char * digest, uint8_t hw_method) {
    int ret = 0;
    ESP_LOGW(TAG, "blinds_stm32_bootloader_ota - stack: %d", uxTaskGetStackHighWaterMark(NULL));

    enable_debug_msg = 1;
    if (!hw_method) {
        // make sure we are in fact communicating with the firmware and update the extended status at the same time 
        if (!blinds_read_status_reg_blocking(EXT_STATUS_REG, 500)) {
            mqtt_publish("node", "stm32ota", "Firmware communication failure!");
            return 0;
        }
        if (!blinds_enter_bootloader()) {
            mqtt_publish("node", "stm32ota", "Warning, firmware did not respond with proper status change!");
            //return 0;
        }
    }

    if (url != NULL) {
        mqtt_publish("node", "stm32ota", "OTA starting!");
    } else {
        mqtt_publish("node", "stm32ota", "bootloader test");
    }

    if (uart_semaphore && xSemaphoreTake(uart_semaphore, portMAX_DELAY)) {

        configFlashUart(STM32_BOOTLOADER_BAUD_RATE); // Reconfigure UART with 8,E,1
        uart_flush(UART_NUM_1);

        if (enterBootLoader(0, STM32F030_BOOTLOADER_DELAY)) {

            queryBootLoader();

            if (url) {
                enable_debug_msg = 0;
                if (flashSTM_from_URL(url, digest, 0, 0, 0) == ESP_OK) {
                    mqtt_publish("node", "stm32ota", "OTA success!");
                    ret = 1;
                } else {
                    mqtt_publish("node", "stm32ota", "OTA failure!");            
                    blinds_motor_status = BLINDS_MOTOR_ERROR;
                }
            } else {
                mqtt_publish("node", "stm32ota", "bootloader test ok!");
            }

            enable_debug_msg = 1;
            endConn(STM32F030_MEMORY_ADDR);

            // wait until the STM32 should be ready for communication
            vTaskDelay( 500 / portTICK_PERIOD_MS );

        } else {
            mqtt_publish("node", "stm32ota_log", "Bootloader didn't respond to SYNC!");
            mqtt_publish("node", "stm32ota", "bootloader failure!");
            blinds_motor_status = BLINDS_MOTOR_ERROR;      
        }

        uart_config(); // Reconfigure UART with 2400 bps 8,N,1

        xSemaphoreGive(uart_semaphore);
    }
    ESP_LOGW(TAG, "blinds_stm32_ota end - stack: %d", uxTaskGetStackHighWaterMark(NULL));

    return ret;
}


void blinds_stm32_init() {
    setFlashPageSize(STM32F030_FLASH_PAGE_SIZE);
    setFlashCallback(&stm32_flash_callback);
    initSTM32_GPIO(RESET_PIN, BOOT0_PIN);
}

