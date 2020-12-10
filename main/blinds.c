#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "uart.h"
#include "iot_helper.h"
#include "blinds.h"

static const char * TAG = "BLINDS";
static const char * UART_TAG = "BLINDS_UART";

#define DEFAULT_POLLING_INTERVAL 500 //ms

typedef enum packet_state_t {
    PACKET_INCOMPLETE = 0,
    PACKET_VALID,
    PACKET_INVALID
} packet_state_t;

QueueHandle_t  blinds_queue = NULL;

#define BLINDS_TASK_CORE 1
#define BLINDS_UART_TASK_CORE 1

#define MAX_VERSION_LENGTH 6

motor_firmware_status_t motor_firmware_status;  // motor firmware status (not detected / original / custom)

char version[MAX_VERSION_LENGTH];

int version_major, version_minor;

static int polling_interval = 0;

blinds_status_t blinds_status = BLINDS_UNKNOWN;
float blinds_motor_pos;	// Position reported by motor unit (between 0-100%). Custom firmware will handle finer granularity (0.0 - 100.0%)
int blinds_battery_status;
float blinds_voltage;
int blinds_speed;  // Current speed in rpm. 
int blinds_target_speed;

static bool diagnostics = false;   // if this is set, more verbose diagnostics data is sent via MQTT

// These are reported only by custom firmware and are used for diagnostics only
int blinds_calibrating;
int blinds_max_length;
int blinds_full_length;
int blinds_motor_status;
int blinds_location;
int blinds_target_location;
int blinds_default_speed;
int blinds_orientation;
int blinds_motor_current;

blinds_direction_t blinds_direction = DIRECTION_STOPPED;		// current direction the motor is moving

float blinds_revs_left; 	// Used when moving with steps and predefined number of revolutions
float blinds_target_position = -1;	// target position (in motor position units e.g. between 0% and 100%)

bool blinds_force_small_steps = false;	// use quieter small movements
bool blinds_override_limits = false;	// use movement commands overriding the upper/lower limits

float blinds_revs;			// Revolutions since last stewise move command. Just for debugging purposes.. 

unsigned long last_poll_timestamp;
unsigned long last_cmd_timestamp;
unsigned long last_move_cmd_timestamp;
unsigned long next_move_cmd_timestamp;
unsigned long last_status_timestamps[MAX_STATUS_REGISTERS];

// 171 (GEAR RATIO) * (13 + 265.0/360)(revolutions) * 4 (interrupt ticks per revolution)
#define FYRTUR_ORIGINAL_FULL_LENGTH 9396

static const char * direction2txt[3] = { "Up", "Stopped", "Down" };

static const char * motor_status2txt[5] = { "Stopped", "Moving", "Stopping", "CalibratingEndPoint", "Error" };

static const char * blinds_status2txt[6] = { "Unknown", "Stopped", "Stopping", "Moving", "MovingInSteps", "Calibrating" }; 

// For more info about the Fyrtur UART protocol, see https://github.com/mjuhanne/fyrtur-motor-board
static const char cmd_status[2] = { 0xcc, 0xcc };
static const char cmd_status2[2] = { 0xcc, 0xcd };
static const char cmd_status3[2] = { 0xcc, 0xce };
static const char cmd_status4[2] = { 0xcc, 0xdd };

static const char cmd_go_to[2] = { 0xdd, 0x00 };  // the second byte is the parameter (0-100)

static const char cmd_up[2] = { 0x0a, 0xdd };
static const char cmd_down[2] = { 0x0a, 0xee };
static const char cmd_up_17[2] = { 0x0a, 0xd }; // 17 degrees. Warning: This may cause continous up movement when used at motor position 0x00!
static const char cmd_down_17[2] = { 0x0a, 0xe }; // 17 degrees
static const char cmd_stop[2] = { 0x0a, 0xcc };

static const char cmd_force_up_90[2] = { 0xfa, 0xd1 };
static const char cmd_force_down_90[2] = { 0xfa, 0xd2 };
static const char cmd_force_up_6[2] = { 0xfa, 0xd3 }; // 6 degrees
static const char cmd_force_down_6[2] = { 0xfa, 0xd4 }; // 6 degrees

static const char cmd_reset_max_length[2] = { 0xfa, 0x00 };
static const char cmd_set_max_length[2] = { 0xfa, 0xee };
static const char cmd_set_full_length[2] = { 0xfa, 0xcc };

static const char cmd_toggle_orientation[2] = { 0xd6, 0x00 };
static const char cmd_reset_orientation[2] = { 0xd5, 0x00 };

// commands supported by custom firmware
static const char cmd_ext_force_down[2] = { 0xfa, 0xda };
static const char cmd_ext_location[2] = { 0xcc, 0xd0 };
static const char cmd_ext_version[2] = { 0xcc, 0xdc };
static const char cmd_ext_status[2] = { 0xcc, 0xde };
static const char cmd_ext_limits[2] = { 0xcc, 0xdf };
static const char cmd_ext_tuning_params[2] = { 0xcc, 0xd3 };
static const char cmd_ext_set_speed[2] = { 0x20, 0x00 };            // second byte is the speed
static const char cmd_ext_set_default_speed[2] = { 0x30, 0x00 };    // second byte is the default speed
static const char cmd_ext_set_minimum_voltage[2] = { 0x40, 0x00 };  // second byte is minimum operating voltage (0x00 = no minimum)

static const char cmd_ext_go_to[2] = { 0x10, 0x00 }; // target position is the lower 4 bits of the 1st byte + 2nd byte (12 bits of granularity)
static const char cmd_ext_set_location[2] = { 0x50, 0x00 }; // location is the lower 4 bits of the 1st byte + 2nd byte (1 sign bit + 11 bits of integer part)
static const char cmd_ext_set_auto_cal[2] = { 0x60, 0x00 }; 
static const char cmd_ext_set_orientation[2] = { 0x61, 0x00 }; 
static const char cmd_ext_set_max_motor_current[2] = { 0x62, 0x00 }; 
static const char cmd_ext_set_stall_detection_timeout[2] = { 0x63, 0x00 }; 
static const char cmd_ext_go_to_location[2] = { 0x70, 0x00 }; // location is the lower 4 bits of the 1st byte + 2nd byte (1 sign bit + 11 bits of integer part)

static const char cmd_ext_debug[2] = { 0xcc, 0xd1 };
static const char cmd_ext_sensor_debug[2] = { 0xcc, 0xd2 };


char * blinds_get_version() {
    return version;
}

motor_firmware_status_t blinds_get_firmware_status() {
    return motor_firmware_status;
}


void blinds_set_diagnostics( bool _diagnostics ) {
    diagnostics = _diagnostics;
}

int blinds_send_cmd_bytes( uint8_t cmd_byte1, uint8_t cmd_byte2 ) {
    static char cmd [] = { 0x00, 0xff, 0x9a, 0x00, 0x00, 0x00};
    cmd[3] = cmd_byte1;
    cmd[4] = cmd_byte2;
    cmd[5] = cmd_byte1 ^ cmd_byte2;
    int txBytes = uart_write( cmd, 6); 
    return txBytes;
}

int blinds_send_cmd( const char * cmd_bytes ) {
    return blinds_send_cmd_bytes( cmd_bytes[0], cmd_bytes[1] );
}

int blinds_queue_cmd( blinds_cmd_t cmd ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = cmd;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;
}

int blinds_queue_cmd_int( blinds_cmd_t cmd, int param1 ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
        msg.cmd = cmd;
        msg.int_param_1 = param1;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     } else {
        return 0;
     }
     return 1;  
}

int blinds_queue_cmd_float( blinds_cmd_t cmd, float param1 ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
        msg.cmd = cmd;
        msg.float_param_1 = param1;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     } else {
        return 0;
     }
     return 1;  
}

void blinds_process_status( int speed, float pos ) {
    if (blinds_speed != speed) {
        blinds_speed = speed;
        blinds_variable_updated(BLINDS_SPEED);
    }
    if (blinds_motor_pos != pos) {
        blinds_motor_pos = pos;
        blinds_variable_updated(BLINDS_POSITION);
    }
    int old_status = blinds_status;
    blinds_direction_t old_direction = blinds_direction;

    if (blinds_target_position != -1) {
        if (blinds_status == BLINDS_MOVING_STEPS) { 
            if ( ( (blinds_direction == DIRECTION_UP) && (blinds_motor_pos <= blinds_target_position ) ) ||
                 ( (blinds_direction == DIRECTION_DOWN) && (blinds_motor_pos >= blinds_target_position ) ) ) {
                ESP_LOGW(TAG, "Reached target! Stopped after %.1f stepwise revolutions..", blinds_revs);
                blinds_status = BLINDS_STOPPED;
                blinds_target_position = -1;
                blinds_revs = 0;
            }
        }
    }
    
    if ( (blinds_status == BLINDS_UNKNOWN) && (blinds_speed>0) ) {
        ESP_LOGW(TAG, "Blinds automatically calibrating...");
        blinds_status = BLINDS_CALIBRATING;
        polling_interval = DEFAULT_POLLING_INTERVAL;
    } else if ( (blinds_status == BLINDS_UNKNOWN) && (blinds_speed==0) ) {
        ESP_LOGI(TAG, "Blinds are stopped...");
        blinds_status = BLINDS_STOPPED;
        blinds_direction = 0;
    } else if (blinds_status == BLINDS_CALIBRATING) {
        if ( (blinds_speed == 0) && (blinds_motor_pos==0) ) {
            ESP_LOGI(TAG, "Blinds calibrated successfully..");
            blinds_status = BLINDS_STOPPED;
            blinds_direction = 0;
            polling_interval = 0;
        }
    } else if ( (blinds_status != BLINDS_STOPPED) && (blinds_status != BLINDS_MOVING_STEPS) && (blinds_speed == 0) && (iot_timestamp() - last_move_cmd_timestamp > 1000) ) {
        ESP_LOGW(TAG, "Blinds stopped at pos %.2f", blinds_motor_pos);
        blinds_status = BLINDS_STOPPED;
        blinds_direction = 0;
        polling_interval = 0;
        blinds_target_position = -1;
    }

    if (blinds_status != old_status) {
        blinds_variable_updated(BLINDS_STATUS);        
    }
    if (blinds_direction != old_direction) {
        blinds_variable_updated(BLINDS_DIRECTION);        
    }
 
    ESP_LOGI(TAG,"STATUS:%s, st_bits 0x%x, %.1fV, SPD %d RPM, POS %.2f (TARGET %.2f), STEPW_REVS %.2f", 
        blinds_status2txt[blinds_status], blinds_battery_status, blinds_voltage, blinds_speed, blinds_motor_pos, 
        blinds_target_position, blinds_revs );
}


void blinds_task_read_status_reg( status_register_t status_reg ) {
	if (status_reg == STATUS_REG_1)
	    blinds_send_cmd( cmd_status );
	else if (status_reg == STATUS_REG_2)
	    blinds_send_cmd( cmd_status2 );
	else if (status_reg == STATUS_REG_3)
	    blinds_send_cmd( cmd_status3 );
	else if (status_reg == STATUS_REG_4)
	    blinds_send_cmd( cmd_status4 );
    else if (status_reg == EXT_LOCATION_REG)
        blinds_send_cmd( cmd_ext_location );
    else if (status_reg == EXT_VERSION_REG)
        blinds_send_cmd( cmd_ext_version );
	else if (status_reg == EXT_STATUS_REG)
	    blinds_send_cmd( cmd_ext_status );
	else if (status_reg == EXT_LIMIT_STATUS_REG)
	    blinds_send_cmd( cmd_ext_limits );
    else if (status_reg == EXT_DEBUG_REG)
        blinds_send_cmd( cmd_ext_debug );
    else if (status_reg == EXT_SENSOR_DEBUG_REG)
        blinds_send_cmd( cmd_ext_sensor_debug );
    else if (status_reg == EXT_TUNING_PARAMS_REG)
        blinds_send_cmd( cmd_ext_tuning_params );
}

bool blinds_task_read_status_reg_blocking( status_register_t status_reg, int ms ) {
	unsigned long now = iot_timestamp();
	blinds_task_read_status_reg(status_reg);
	while ( (iot_timestamp() - now < ms) && (last_status_timestamps[status_reg] < now) ) {
		vTaskDelay( 20 / portTICK_PERIOD_MS );
	}
	if (last_status_timestamps[status_reg] < now)
		return false;
	return true;
}


int blinds_task_move_step() {
    float revs = 0;

	if ( (blinds_revs_left>0) || ( (blinds_target_position != -1) && (blinds_motor_pos != blinds_target_position)) ) {
		// let's continue if more revolutions are left or there's specific target position set that hasn't been reached yet

		if (iot_timestamp() >= next_move_cmd_timestamp) {

	        if ( blinds_override_limits && (blinds_revs_left>=(float)90/360) ) {
	            if (blinds_direction == DIRECTION_UP) {
	                blinds_send_cmd( cmd_force_up_90 );
	            } else {
	                blinds_send_cmd( cmd_force_down_90 );
	            }
	            revs = (float)90/360;
	        } else if ( blinds_override_limits || blinds_force_small_steps ) {
	        	// this is the smaller forceable movement command which is a bit quieter than continous movement (on original firmware)
	        	// With newer FW, just set slower movement speed and use continous movement commands
	            if (blinds_direction == DIRECTION_UP) {
	                blinds_send_cmd( cmd_force_up_6 );
	            } else {
	                blinds_send_cmd( cmd_force_down_6 );
	            }
	            revs = (float)6/360;
	        } else if ( !blinds_override_limits ) {
	            if (blinds_direction == DIRECTION_UP) {
	                blinds_send_cmd( cmd_up_17 );
	            } else {
	                blinds_send_cmd( cmd_down_17 );
	            }
	            revs = (float)17/360;
	        }

			last_move_cmd_timestamp = iot_timestamp();
			// Original firmware cannot handle frequent move commands so lets
			// try to estimate when we are allowed to send the next one
			next_move_cmd_timestamp = iot_timestamp() + 300 + revs*3000;

			blinds_task_read_status_reg(STATUS_REG_1);

	        ESP_LOGI(TAG,"Stepwise movement of %.2f degrees", revs*360);

            blinds_revs += revs;

	        blinds_revs_left -= revs;

	    }
	} else {
		ESP_LOGI(TAG,"End of stepwise movement (motor pos %.2f, target pos %.2f, revs_left %f)", blinds_motor_pos, blinds_target_position, blinds_revs_left);
		blinds_status = BLINDS_STOPPED;
	    blinds_target_position = -1;
	}
    return 1;
}


/*
	- force_small_steps flag uses only small steps (a bit quieter operation)
	- override_limits flag allows movement outside lower limit and uses (for now) only small steps
	- target_position: if unspecified (-1), will be either upper or lower limit depending on the direction. Ignored when moving by specific revolutions
*/
int blinds_task_move( int direction, float revs, float target_position, bool force_small_steps, bool override_limits  ) {
	ESP_LOGI(__func__,"Move: %s - %.2f revs, target %.2f, force_small_steps %d, force_mov %d", direction2txt[direction+1], revs, target_position, force_small_steps, override_limits);
    blinds_direction_t old_direction = blinds_direction;
    int old_target_pos = blinds_target_position;
    blinds_status_t old_status = blinds_status;
    if ( (revs>0) || force_small_steps || ( (motor_firmware_status == ORIGINAL_FW) && override_limits) ) {
    	// Here we use smaller steps because of the following reasons:
    	//	- we want to turn specific amount of revolutions
        //  - using small steps was requested
    	//	- we need to move outside lower limit (force-flag) when using original firmware
    	ESP_LOGI(__func__,"Starting stepwise movement");
        blinds_direction = direction;
        blinds_revs_left = revs;
        blinds_revs = 0;        
        if (revs == -1) {
        	if (target_position == -1) {
	        	if (direction == DIRECTION_UP)
	        		blinds_target_position = 0;
	        	else 
	        		blinds_target_position = 100;
        	} else {
        		blinds_target_position = target_position;
        	}
        } else {        	
        	blinds_target_position = -1; // we don't use motors position as target but move specific amount of revolutions
        }
        polling_interval = DEFAULT_POLLING_INTERVAL;
        blinds_override_limits = override_limits;
        blinds_force_small_steps = force_small_steps;
        blinds_status = BLINDS_MOVING_STEPS;
        next_move_cmd_timestamp = iot_timestamp(); // next step is now
        blinds_task_move_step();
    } else {
    	ESP_LOGI(__func__,"Starting continuous movement");
    	// continous fast (noisier in default fw) movement
    	if (target_position != -1) {
    		char cmd[2];
    		if (motor_firmware_status == CUSTOM_FW) {
    			// extended GO_TO cmd_byte format in bits: CCCCWWWW WWWWDDDD, where CCCC=0xba(13), W denotes whole number part (0-100) and D decimal part of the target position
    			cmd[0] = cmd_ext_go_to[0];
    			uint16_t pos = target_position * 16; // convert float to int
    			cmd[0] |= (pos>>8);  // upper 4 bits
    			cmd[1] = pos & 0xff;  // lower 8 bits
                blinds_send_cmd( cmd );
    		} else {
    			cmd[0] = cmd_go_to[0];
    			cmd[1] = (uint8_t)target_position;
                blinds_send_cmd( cmd );
    		}
            blinds_target_position = target_position;
    	} else {
	        if (direction == DIRECTION_UP) {
	            if (blinds_motor_pos > 0.1) {
	                blinds_send_cmd( cmd_up );
	                blinds_direction = DIRECTION_UP;
		            blinds_target_position = 0;
	            } else {
	                ESP_LOGE(TAG,"Already up!");
	                return 0;
	            }
	        } else {
                if ( (motor_firmware_status == CUSTOM_FW) && (override_limits) ) {
                    blinds_send_cmd( cmd_ext_force_down );            
                    blinds_target_position = 110;   // just so that we don't stop the movement before down button is released
                } else {
    	            blinds_send_cmd( cmd_down );            
    	            blinds_target_position = 100;
    	        }
                blinds_direction = DIRECTION_DOWN;
            }
	    }
        polling_interval = DEFAULT_POLLING_INTERVAL;
        blinds_status = BLINDS_MOVING;
        blinds_force_small_steps = false;
        blinds_override_limits = false;
        last_move_cmd_timestamp = iot_timestamp();
    }

    if (blinds_status != old_status) {
        blinds_variable_updated(BLINDS_STATUS);        
    }
    if (blinds_direction != old_direction) {
        blinds_variable_updated(BLINDS_DIRECTION);        
    }
    if (blinds_target_position != old_target_pos) {
        blinds_variable_updated(BLINDS_TARGET_POSITION);        
    }

    return 1;
}


int blinds_move( blinds_direction_t direction, float revs, bool force_small_steps, bool override_limits ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_move;
    	msg.int_param_1 = direction;
    	msg.float_param_1 = revs;
    	msg.int_param_2 = force_small_steps;
    	msg.int_param_3 = override_limits;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;
}


int blinds_task_go_to( float target_position, bool force_small_steps ) {
	int direction;
	if (blinds_motor_pos < target_position) {
		direction=DIRECTION_DOWN;
	} else {
		direction=DIRECTION_UP;
	}
	blinds_task_move(direction, -1, target_position, force_small_steps, false);
	return 1;
}


int blinds_go_to( float position, bool force_small_steps ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_go_to;
    	msg.float_param_1 = position;
    	msg.int_param_1 = force_small_steps;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;
}

int blinds_task_set_location( int location ) {
    char cmd[2];
    location = location >> 1;  // There is only room for 12 bits of data, so we omit 1 least-significant bit
    cmd[0] = cmd_ext_set_location[0];
    cmd[0] |= ( (location>>8) & 0xf);  // upper 4 bits
    cmd[1] = location & 0xff;  // lower 8 bits
    blinds_send_cmd( cmd );
    return 1;
}


int blinds_set_location( int location ) {
    return blinds_queue_cmd_int(blinds_cmd_set_location, location);
}

int blinds_task_go_to_location( int location ) {
    char cmd[2];    
    location = location >> 1;  // There is only room for 12 bits of data, so we omit 1 least-significant bit
    cmd[0] = cmd_ext_go_to_location[0];
    cmd[0] |= ( (location>>8) & 0xf);  // upper 4 bits
    cmd[1] = location & 0xff;  // lower 8 bits
    blinds_send_cmd( cmd );
    return 1;
}


int blinds_go_to_location( int location ) {
    return blinds_queue_cmd_int(blinds_cmd_go_to_location, location);
}


int blinds_task_stop() {
    blinds_send_cmd( cmd_stop );
    blinds_status = BLINDS_STOPPING;
    blinds_revs_left = 0;
    /*
    polling_interval=0;
    */
    return 1;
}

int blinds_stop() {
	return blinds_queue_cmd(blinds_cmd_stop);
}


int blinds_task_reset_max_length() {
    ESP_LOGI(TAG,"Reset maximum blind length and calibrate..");
    blinds_send_cmd( cmd_reset_max_length );
    // delay until motor has settled
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int i=0;
    while ( (blinds_motor_pos != 0x32) && (i<10) ) {
        blinds_task_read_status_reg_blocking(STATUS_REG_1, 500);
        i++;
    }
    if (blinds_motor_pos == 0x32) {
        ESP_LOGI(TAG,"Maximum curtain length reseted. Doing the calibration now by winding up the curtain..");
        blinds_task_move(DIRECTION_UP, -1, -1, false, false);
        blinds_status = BLINDS_CALIBRATING;

        if (motor_firmware_status == CUSTOM_FW) {
            // read back the max length
            blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
        }
    } else {
        ESP_LOGE(TAG,"Maximum length reset unsuccessful! (position %.2f != 50!)", blinds_motor_pos);
        blinds_status = BLINDS_STOPPED;
    }
    return 1;
}

int blinds_reset_max_length() {
	return blinds_queue_cmd(blinds_cmd_reset_max_length);
}

int blinds_task_set_max_length() {
    ESP_LOGI(TAG,"Setting maximum curtain length..");
    blinds_send_cmd( cmd_set_max_length );
    // delay until motor has settled
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg_blocking(STATUS_REG_1, 500);
    if (blinds_motor_pos == 0x64) {
        ESP_LOGI(TAG,"Maximum curtain length has been set to current position.");
    } else {
        ESP_LOGE(TAG,"Error setting maximum curtain length (position %.2f != 100!)", blinds_motor_pos);
    }
    if (motor_firmware_status == CUSTOM_FW) {
        // read back the max length
        blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
    }
    return 1;
}

int blinds_set_max_length() {
	return blinds_queue_cmd(blinds_cmd_set_max_length);
}

int blinds_task_set_full_length() {
    ESP_LOGI(TAG,"Setting full curtain length..");
    blinds_send_cmd( cmd_set_full_length );
    // delay until motor has settled
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg_blocking(STATUS_REG_1, 500);
    if (blinds_motor_pos == 0x64) {
        ESP_LOGI(TAG,"Full curtain length has been set to current position.");
    } else {
        ESP_LOGE(TAG,"Error full curtain length (position %.2f != 100!)", blinds_motor_pos);
    }
    if (motor_firmware_status == CUSTOM_FW) {
        // read back the max and full lengths
        blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
    }
    return 1;
}

int blinds_set_full_length() {
	return blinds_queue_cmd(blinds_cmd_set_full_length);
}

int blinds_task_reset_full_length() {
    if (blinds_get_firmware_status() == CUSTOM_FW) {
        ESP_LOGI(TAG,"Reset to FULL blind length and then calibrate..");

        // Set current location to original full length..
        blinds_task_set_location(FYRTUR_ORIGINAL_FULL_LENGTH);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // .. then set current location as the full curtain length
        blinds_task_set_full_length();
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Then reset max length and do the calibration
        blinds_task_reset_max_length();

    } else {
        ESP_LOGI(TAG,"Cannot reset FULL blind length with original firmware! Doing only max length reset");
        blinds_task_reset_max_length();
    }
    return 1;
}

int blinds_reset_full_length() {
    return blinds_queue_cmd(blinds_cmd_reset_full_length);
}


int blinds_read_status_reg( status_register_t status_reg ) {
    return blinds_queue_cmd_int(blinds_cmd_status, status_reg);
}

int blinds_read_ext_status_reg() {
	return blinds_queue_cmd(blinds_cmd_ext_status);
}

int blinds_send_raw( uint8_t cmd_byte1, uint8_t cmd_byte2 ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_send_raw;
    	msg.int_param_1 = cmd_byte1;
    	msg.int_param_2 = cmd_byte2;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;	
}

int blinds_task_set_speed(int speed) {
	char cmd[2];
	if (motor_firmware_status == CUSTOM_FW) {
        blinds_target_speed = speed;
		cmd[0] = cmd_ext_set_speed[0];
		cmd[1] = (uint8_t)speed;
        blinds_send_cmd( cmd );
    }
    return 1;
}

int blinds_set_speed(int speed) {
    return blinds_queue_cmd_int(blinds_cmd_set_speed, speed);
}

int blinds_task_set_default_speed(int speed) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_default_speed[0];
        cmd[1] = (uint8_t)speed;
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_default_speed(int speed) {
    return blinds_queue_cmd_int(blinds_cmd_set_default_speed, speed);
}

int blinds_task_set_minimum_voltage(float voltage) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_minimum_voltage[0];
        cmd[1] = (uint8_t)(voltage*16);
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_minimum_voltage(float voltage) {
    return blinds_queue_cmd_float(blinds_cmd_set_minimum_voltage, voltage);
}

int blinds_task_set_max_motor_current(int max_current) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_max_motor_current[0];
        cmd[1] = (uint8_t)max_current/8;
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_max_motor_current(int max_current) {
    return blinds_queue_cmd_int(blinds_cmd_set_max_motor_current, max_current);
}

int blinds_task_set_stall_detection_timeout(int stall_detection_timeout) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_stall_detection_timeout[0];
        cmd[1] = (uint8_t)stall_detection_timeout/8;
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_stall_detection_timeout(int stall_detection_timeout) {
    return blinds_queue_cmd_int(blinds_cmd_set_stall_detection_timeout, stall_detection_timeout);
}


int blinds_task_set_auto_cal(bool enabled) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_auto_cal[0];
        cmd[1] = enabled;
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_auto_cal(bool enabled) {
    return blinds_queue_cmd_int(blinds_cmd_set_auto_cal, enabled);
}

int blinds_task_set_orientation(blinds_orientation_t orientation) {
    char cmd[2];
    if (motor_firmware_status == CUSTOM_FW) {
        cmd[0] = cmd_ext_set_orientation[0];
        cmd[1] = orientation;
        blinds_send_cmd( cmd ); 

        vTaskDelay(100 / portTICK_PERIOD_MS);

        // read back the setting
        blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
    }
    return 1;
}

int blinds_set_orientation( blinds_orientation_t orientation) {
    return blinds_queue_cmd_int(blinds_cmd_set_orientation, orientation);
}

int blinds_task_reset_orientation() {
    blinds_send_cmd( cmd_reset_orientation );
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_STATUS_REG);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_LOCATION_REG);
    return 1;
}

int blinds_reset_orientation() {
    return blinds_queue_cmd(blinds_cmd_reset_orientation);
}

int blinds_task_toggle_orientation() {
    blinds_send_cmd( cmd_toggle_orientation );
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_LIMIT_STATUS_REG);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_STATUS_REG);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg(EXT_LOCATION_REG);
    return 1;
}

int blinds_toggle_orientation() {
    return blinds_queue_cmd(blinds_cmd_toggle_orientation);
}

int blinds_get_speed() {
	return blinds_speed;
}

int blinds_get_target_speed() {
    return blinds_target_speed;
}

float blinds_get_position() {
	return blinds_motor_pos;
}

float blinds_get_voltage() {
    return blinds_voltage;
}

int blinds_get_full_length() {
    return blinds_full_length;
}

int blinds_get_max_length() {
    return blinds_max_length;
}

int blinds_get_location() {
    return blinds_location;
}

int blinds_get_target_location() {
    return blinds_target_location;
}

const char * blinds_get_motor_status_str() {
    return (blinds_motor_status <= 4 ? motor_status2txt[blinds_motor_status] : "ERR");
}

int blinds_get_calibration_status() {
    return blinds_calibrating;
}

int blinds_get_orientation() {
    return blinds_orientation;
}

int blinds_get_current() {
    return blinds_motor_current;
}


blinds_status_t blinds_get_status() {
    return blinds_status;
}

blinds_direction_t blinds_get_direction() {
    return blinds_direction;
}

const char * blinds_get_status_str() {
    return blinds_status2txt[blinds_status];
}
const char * blinds_get_direction_str() {
    return direction2txt[blinds_direction + 1];
}

int blinds_get_target_position() {
    return blinds_target_position;
}


void blinds_process_status_reg_1( int battery_status, int voltage, int speed, int pos ) {
    blinds_battery_status = battery_status;
    float f_voltage = (float)voltage/30;
    if (blinds_voltage != f_voltage) {
        blinds_voltage = f_voltage;
        blinds_variable_updated(BLINDS_VOLTAGE);
    }

    blinds_process_status( speed, pos );    // speed and position will be processed separately
    last_status_timestamps[STATUS_REG_1] = iot_timestamp();
}


void blinds_process_limit_status_reg( int calibrating, int orientation, int max_length, int full_length ) {
	ESP_LOGI(UART_TAG,"EXT_LIMIT_STAT: calibrating: %d, orientation %d, max_length %d, full_length %d", calibrating, orientation, max_length, full_length );
    if (blinds_calibrating != calibrating) {
        blinds_calibrating = calibrating;   
        blinds_variable_updated(BLINDS_CALIBRATING_STATUS);     
    }
    if (blinds_orientation != orientation) {
        blinds_orientation = orientation;   
        blinds_variable_updated(BLINDS_ORIENTATION);     
    }
    if (blinds_max_length != max_length) {
        blinds_max_length = max_length;
        blinds_variable_updated(BLINDS_MAX_LEN);
    }
    if (blinds_full_length != full_length) {
        blinds_full_length = full_length;
        blinds_variable_updated(BLINDS_FULL_LEN);
    }

	last_status_timestamps[EXT_LIMIT_STATUS_REG] = iot_timestamp();
}

/*
 * Reports position with greater resolution than original firmware
 * Motor current and status is just for debugging purposes
 */
void blinds_process_ext_status_reg( int status, int current, int speed, float position) {
	ESP_LOGI(UART_TAG,"EXT_STAT: status: %s, current %d mA, speed %d RPM, position %.2f", 
        blinds_get_motor_status_str(), current, speed, position);

    blinds_process_status( speed, position );   // speed and position will be processed separately

    if (blinds_motor_status != status) {
        blinds_motor_status = status;
        blinds_variable_updated(BLINDS_MOTOR_STATUS);
    }
    if (blinds_motor_current != current) {
        blinds_motor_current = current;
        blinds_variable_updated(BLINDS_MOTOR_CURRENT);
    }
	last_status_timestamps[EXT_STATUS_REG] = iot_timestamp();
}


void blinds_process_tuning_params_reg( int slowdown_factor, int min_slowdown_speed, int stall_detection_timeout, int max_motor_current, int unused) {
    ESP_LOGI(UART_TAG,"TUNING_PARAMS: slowdown_factor %d, min_slowdown_speed %d, stall_detection_timeout %d, max_motor_current %d mA, unused %d",
        slowdown_factor, min_slowdown_speed, stall_detection_timeout, max_motor_current, unused);
    last_status_timestamps[EXT_TUNING_PARAMS_REG] = iot_timestamp();
}


void blinds_process_ext_location_reg( int location, int target_location ) {
    ESP_LOGI(UART_TAG,"EXT_LOCATION: loc %d, target_loc %d", location, target_location);
    last_status_timestamps[EXT_LOCATION_REG] = iot_timestamp();
    if (blinds_location != location) {
        blinds_location = location;
        blinds_variable_updated(BLINDS_LOCATION);
    }
    if (blinds_target_location != target_location) {
        blinds_target_location = target_location;
        blinds_variable_updated(BLINDS_TARGET_LOCATION);
    }
}

void blinds_process_ext_version_reg( int version_major, int version_minor, int voltage_check, int default_speed ) {
    ESP_LOGI(UART_TAG,"VERSION: version %d.%d. Minimum voltage: %.2f. Default speed %d", 
        version_major, version_minor, (float)voltage_check/16, default_speed);
    snprintf(version, MAX_VERSION_LENGTH, "%d.%d", version_major, version_minor);
    blinds_default_speed = default_speed;
    last_status_timestamps[EXT_VERSION_REG] = iot_timestamp();
}

void blinds_process_status_reg_2( int status_index, uint8_t * bytes, int len ) {
	if (len == 5) {
		ESP_LOGI(UART_TAG,"STAT(%d): 0x%x 0x%x 0x%x 0x%x 0x%x", status_index+1,
				bytes[0], bytes[1], bytes[2], bytes[3], bytes[4] );
	} else if (len == 2) {
    	ESP_LOGI(UART_TAG,"STAT(%d): 0x%x 0x%x", status_index+1, bytes[0], bytes[1]);
	} else {
		ESP_LOGE(UART_TAG,"STAT(%d): unexpected length!", status_index+1);
	}
	last_status_timestamps[status_index] = iot_timestamp();
}


/*
  For detailed information about communication protocol, see Fyrtur motor board firmware documentation 
  (https://github.com/mjuhanne/fyrtur-motor-board)
*/
void blinds_uart_task(void *pvParameter) {

    ESP_LOGI(UART_TAG,"Running on core #%d", xPortGetCoreID());
    int totRxBytes = 0;
    int expectedBytes = 3;
    int checksum;

    // Since the received data packet can be 8 or 9 bytes long, in this loop we first just read 3 bytes (the header) and then anticipate
    // the additional number of bytes. If header is unknown, we assume the protocol is out of sync/corrupted, so we skip the first byte
    // of the header and read one more byte and repeat until a valid header is received.
    while (1) {
		static uint8_t data[9];
		packet_state_t packet_state = PACKET_INCOMPLETE;
		const int rxBytes = uart_read( &data[totRxBytes], expectedBytes-totRxBytes, 1000 );
		totRxBytes += rxBytes;
		if (totRxBytes==expectedBytes) {

            //ESP_LOGD(UART_TAG, "rcv - Stack: %d", uxTaskGetStackHighWaterMark(NULL));

		    if ( (data[0]==0) && (data[1]==0xff) && (data[2]==0xd8) ) {
		    	// Status #1
		    	if (totRxBytes != 8) {
		    		expectedBytes = 8;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
			        if (checksum == data[7]) {
			    	// STATUS
			            blinds_process_status_reg_1( data[3], data[4], data[5], data[6] );
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }
			    }

			} else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd6) ) {
	        	// Status #2
		    	if (totRxBytes < 9) {
		    		expectedBytes = 9;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
			        if (checksum == data[8]) {
			        	blinds_process_status_reg_2(STATUS_REG_2, &data[3], 5);
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }

		    	}

		    } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd4) ) {
	        	// Status #3
		    	if (totRxBytes < 9) {
		    		expectedBytes = 9;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
			        if (checksum == data[8]) {
			        	blinds_process_status_reg_2(STATUS_REG_3, &data[3], 5);
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }

		    	}

		    } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd9) ) {
	        	// Status #4
		    	if (totRxBytes < 6) {
		    		expectedBytes = 6;
		    	} else {
			        checksum = data[3] ^ data[4];
			        if (checksum == data[5]) {
			        	blinds_process_status_reg_2(STATUS_REG_4, &data[3], 2);
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }

		    	}
            } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd0) ) {
                // EXTENDED VERSION
                if (totRxBytes != 8) {
                    expectedBytes = 8;
                } else {
                    checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
                    if (checksum == data[7]) {
                        blinds_process_ext_version_reg( data[3], data[4], data[5], data[6] );
                        packet_state = PACKET_VALID;
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }
            } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd1) ) {
                // EXTENDED LOCATION
                if (totRxBytes != 8) {
                    expectedBytes = 8;
                } else {
                    checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
                    if (checksum == data[7]) {
                        blinds_process_ext_location_reg( data[3]*256 + data[4], data[5]*256 + data[6]  );
                        packet_state = PACKET_VALID;
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }
            } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd2) ) {
                // DEBUG BYTES
                if (totRxBytes != 9) {
                    expectedBytes = 9;
                } else {
                    checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
                    if (checksum == data[8]) {
                        ESP_LOGW(UART_TAG,"debug bytes: unused %d, dir_error %d, cal %d, stopped_ticks %d, unused %d ",  data[3], data[4], data[5], data[6], data[7]);
                        packet_state = PACKET_VALID;
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }
            } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd3) ) {
                // SENSOR DEBUG BYTES
                if (totRxBytes != 9) {
                    expectedBytes = 9;
                } else {
                    checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
                    if (checksum == data[8]) {
                        ESP_LOGW(UART_TAG,"sensor debug bytes: hall1ticks %d, hall2ticks %d, unused %d ", data[3]*256+data[4], data[5]*256+data[6], data[7]);
                        packet_state = PACKET_VALID;
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }
		    } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xda) ) {
		        // EXTENDED STATUS
		    	if (totRxBytes != 9) {
		    		expectedBytes = 9;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
			        if (checksum == data[8]) {
			        	blinds_process_ext_status_reg( data[3], data[4]*8, data[5], data[6] + (float)data[7]/256 );
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }
		        }
		    } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xdb) ) {
		        // LIMIT STATUS
		    	if (totRxBytes != 9) {
		    		expectedBytes = 9;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
			        if (checksum == data[8]) {
			            packet_state = PACKET_VALID;
			            blinds_process_limit_status_reg( data[3]&1, (data[3]>>1)&1, data[4]*256 + data[5], data[6]*256 + data[7] );
			        } else {
			        	packet_state = PACKET_INVALID;
			        }
		        }
            } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xd5) ) {
                // TUNING PARAMETERS
                if (totRxBytes != 9) {
                    expectedBytes = 9;
                } else {
                    checksum = data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
                    if (checksum == data[8]) {
                        packet_state = PACKET_VALID;
                        blinds_process_tuning_params_reg( data[3], data[4], data[5]*8, data[6]*8, data[7] );
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }
		    } else if ( (data[0]==0x00) && (data[1]==0xab) && (data[2]==0xba) ) {
		    	// Response to PING
		    	if (totRxBytes != 8) {
		    		expectedBytes = 8;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
			        if (checksum == data[7]) {
			        	if ( (data[3]==0x00) && (data[4]==0xff) && (data[5]==0x9a) ) {
	    		        	ESP_LOGI(UART_TAG,"VALID PING:  id 0x%x", data[6]);
				            packet_state = PACKET_VALID;
	        			} else {
	            			ESP_LOGE(UART_TAG,"INVALID PING 0x%x 0x%x 0x%x , id 0x%x", data[3], data[4], data[5], data[6]);
	        			}
			        } else {
			        	packet_state = PACKET_INVALID;
			        }
			    }

		    } else if ( (data[0]==0xde) && (data[1]==0xad) ) {
		    	// Error message (Motor module received incomplete packet). Used for debugging only for now
		    	if (totRxBytes != 8) {
		    		expectedBytes = 8;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
			        if (checksum == data[7]) {
			            ESP_LOGE(UART_TAG,"ERROR MSG. Motor received %d bytes. First 4 bytes: 0x%x 0x%x 0x%x 0x%x", data[2], data[3], data[4], data[5], data[6]);
			            packet_state = PACKET_VALID;
			        } else {
			        	packet_state = PACKET_INVALID;
			        }
		        }
		    } else {
		    	// UNKNOWN HEADER!
	        	// Try omitting first byte and then reading one more byte in case we are out of sync
	        	for (int i=0;i<2;i++)
	        		data[i] = data[i+1];
	        	totRxBytes = 2;
		    } 

		    if (packet_state == PACKET_INVALID) {
	            ESP_LOGE(UART_TAG,"Invalid checksum: %d != %d", checksum, data[totRxBytes-1]);
	            ESP_LOG_BUFFER_HEXDUMP(UART_TAG, data, totRxBytes, ESP_LOG_ERROR);
	        }
		    
		    if (packet_state != PACKET_INCOMPLETE) {
		    	// reset packet state
		    	totRxBytes = 0;
		    	expectedBytes = 3;
		    }

		} else {
		    if (totRxBytes==0) {
		        //ESP_LOGE(TAG,"read_status timeout!");
		    } else {
		        ESP_LOGE(UART_TAG,"invalid data length (%d)!",totRxBytes);
		        ESP_LOG_BUFFER_HEXDUMP(UART_TAG, data, totRxBytes, ESP_LOG_ERROR);
		        totRxBytes = 0;      
		    }
		    expectedBytes = 3;
		}

        vTaskDelay(10 / portTICK_PERIOD_MS);    		
    }
}




void blinds_task(void *pvParameter) {

    ESP_LOGI(TAG,"Running on core #%d", xPortGetCoreID());

    blinds_msg msg;

    while (1) {

    	if (blinds_queue != NULL) {
            if (xQueueReceive(blinds_queue,&msg,(TickType_t )(20/portTICK_PERIOD_MS))) {
                ESP_LOGW(TAG, "queue - Stack: %d", uxTaskGetStackHighWaterMark(NULL));

            	switch (msg.cmd) {
            		case blinds_cmd_move: {
    	        			ESP_LOGI(TAG,"Move (direction %s, revs %.2f, force_small_steps-flag %d, override_limits: %d", 
                                direction2txt[msg.int_param_1+1], msg.float_param_1, msg.int_param_2, msg.int_param_3);
    	        			blinds_task_move(msg.int_param_1, msg.float_param_1, -1, msg.int_param_2, msg.int_param_3);
	            		}
        	    		break;
            		case blinds_cmd_go_to: {
    	        			ESP_LOGI(TAG,"Move to position (pos %.2f, force_small_steps-flag: %d)", msg.float_param_1, msg.int_param_1);
    	        			blinds_task_go_to(msg.float_param_1, msg.int_param_1);
	            		}
        	    		break;
            		case blinds_cmd_set_speed: {
            				if (motor_firmware_status == CUSTOM_FW) {
	    	        			ESP_LOGI(TAG,"Setting speed to %d rpm", msg.int_param_1);
    		        			blinds_task_set_speed(msg.int_param_1);
            				} else {
	    	        			ESP_LOGE(TAG,"Setting speed not supported on original firmware!");            					
            				}
	            		}
        	    		break;
                    case blinds_cmd_set_default_speed: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting default speed to %d rpm", msg.int_param_1);
                                blinds_task_set_default_speed(msg.int_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting defaut speed not supported on original firmware!");                              
                            }
                        }
                        break;
            		case blinds_cmd_stop: {
            				ESP_LOGI(TAG,"Stop");
            				blinds_task_stop();
            			}
            			break;
            		case blinds_cmd_reset_max_length: {
           	 				ESP_LOGI(TAG,"Reset max curtain length");
           	 				blinds_task_reset_max_length();
            			}
            			break;
                    case blinds_cmd_set_location: {
                            ESP_LOGI(TAG,"Set location to %d", msg.int_param_1);
                            blinds_task_set_location(msg.int_param_1);
                        }
                        break;
                    case blinds_cmd_go_to_location: {
                            ESP_LOGI(TAG,"Go to location to %d", msg.int_param_1);
                            blinds_task_go_to_location(msg.int_param_1);
                        }
                        break;
            		case blinds_cmd_set_max_length: {
	            			ESP_LOGI(TAG,"Set maximum curtain length");
	            			blinds_task_set_max_length();
    	        		}
        	    		break;
            		case blinds_cmd_set_full_length: {
            				ESP_LOGI(TAG,"Set full curtain length");
	            			blinds_task_set_full_length();
            			}
            			break;
                    case blinds_cmd_reset_full_length: {
                            ESP_LOGI(TAG,"Reset full and max curtain lengths");
                            blinds_task_reset_full_length();
                        }
                        break;
                    case blinds_cmd_reset_orientation: {
                            ESP_LOGI(TAG,"Reset orientation");
                            blinds_task_reset_orientation();
                        }
                        break;
                    case blinds_cmd_toggle_orientation: {
                            ESP_LOGI(TAG,"Toggle orientation");
                            blinds_task_toggle_orientation();
                        }
                        break;
            		case blinds_cmd_status: {
            				ESP_LOGI(TAG,"Read status reg %d", (int)msg.int_param_1);
	            			blinds_task_read_status_reg(msg.int_param_1);
            			}
            			break;
            		case blinds_cmd_send_raw: {
    	        			ESP_LOGI(TAG,"Send raw command (0x%x 0x%x)", msg.int_param_1, msg.int_param_2);
    	        			blinds_send_cmd_bytes(msg.int_param_1, msg.int_param_2);
	            		}
        	    		break;
                    case blinds_cmd_set_minimum_voltage: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting minimum voltage to %.1f rpm", msg.float_param_1);
                                blinds_task_set_minimum_voltage(msg.float_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting minimum voltage not supported on original firmware!");                              
                            }
                        }
                        break;
                    case blinds_cmd_set_auto_cal: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting auto calibration to %d", msg.int_param_1);
                                blinds_task_set_auto_cal(msg.int_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting auto calibration not supported on original firmware!");                              
                            }
                        }
                        break;
                    case blinds_cmd_set_orientation: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting orientation to %d", msg.int_param_1);
                                blinds_task_set_orientation(msg.int_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting orientation not supported on original firmware!");                              
                            }
                        }
                        break;
                    case blinds_cmd_set_stall_detection_timeout: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting hall sensor timeout to %d ms", msg.int_param_1);
                                blinds_task_set_stall_detection_timeout(msg.int_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting hall sensor timeout not supported on original firmware!");                              
                            }
                        }
                        break;
                    case blinds_cmd_set_max_motor_current: {
                            if (motor_firmware_status == CUSTOM_FW) {
                                ESP_LOGI(TAG,"Setting max motor current to %d mA", msg.int_param_1);
                                blinds_task_set_max_motor_current(msg.int_param_1);
                            } else {
                                ESP_LOGE(TAG,"Setting max motor current not supported on original firmware!");                              
                            }
                        }
                        break;
            		default: {
    	        			ESP_LOGE(TAG,"Unknown command! (%d)",msg.cmd);
            			}
            			break;
            	}
            }

            if (blinds_status == BLINDS_MOVING_STEPS) {
            	blinds_task_move_step();
            }

	        if ( (polling_interval) && (blinds_status != BLINDS_STOPPED) && (iot_timestamp() - last_poll_timestamp  > polling_interval)) {
                last_poll_timestamp = iot_timestamp();
                if (motor_firmware_status == CUSTOM_FW) {
                    blinds_task_read_status_reg(EXT_STATUS_REG);
                    if (diagnostics) {
                        //vTaskDelay(20 / portTICK_PERIOD_MS);
                        //blinds_task_read_status_reg(EXT_LOCATION_REG);
                    }
                } else {
                    blinds_task_read_status_reg(STATUS_REG_1);                    
                }
	        }

    	} else {
            vTaskDelay(500 / portTICK_PERIOD_MS);    		
    	}

    }
}



void blinds_init() {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    blinds_queue = xQueueCreate( 10, sizeof( blinds_msg  ) );

    init_uart();    

#ifdef ESP32
    xTaskCreatePinnedToCore(&blinds_task, "blinds_task", 4096, NULL, 5, NULL, BLINDS_TASK_CORE);
    xTaskCreatePinnedToCore(&blinds_uart_task, "blinds_uart_task", 4096, NULL, 5, NULL, BLINDS_UART_TASK_CORE);
#else
    xTaskCreatePinnedToCore(&blinds_task, "blinds_task", 1024, NULL, 5, NULL, BLINDS_TASK_CORE);
    xTaskCreatePinnedToCore(&blinds_uart_task, "blinds_uart_task", 1024, NULL, 5, NULL, BLINDS_UART_TASK_CORE);
#endif

    // Send dummy status query message (since motor unit might not receive the 1st byte correctly after power on)
    blinds_task_read_status_reg_blocking(STATUS_REG_1, 1000);

    ESP_LOGI(TAG,"Checking motor module...");
    if (blinds_task_read_status_reg_blocking(STATUS_REG_1, 5000)) {

        if (blinds_task_read_status_reg_blocking(EXT_STATUS_REG, 1000)) {
            ESP_LOGI(TAG,"Custom Fyrtur motor firmware detected!");

            blinds_task_read_status_reg_blocking(EXT_VERSION_REG, 1000);
            blinds_task_read_status_reg_blocking(EXT_LIMIT_STATUS_REG, 1000);

            motor_firmware_status = CUSTOM_FW;
        } else {
            ESP_LOGI(TAG,"Original Fyrtur motor firmware detected");
            motor_firmware_status = ORIGINAL_FW;
        }


    } else {
        ESP_LOGE(TAG,"Fyrtur motor not connected!");
        motor_firmware_status = MOTOR_NOT_DETECTED;
    }

}

