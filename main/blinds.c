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

/*
    Custom motor firmware allows:
        - setting motor speed
        - higher granularity in motor position (and target position)
        - quering additional variables (resetting-flag, maximum and full curtain lengths)
    See https://github.com/mjuhanne/fyrtur-motor-board
*/
bool custom_fw = false;
char version[MAX_VERSION_LENGTH];

int version_major, version_minor;

static int polling_interval = 0;

blinds_status_t blinds_status = BLINDS_UNKNOWN;
float blinds_motor_pos;	// Position reported by motor unit (between 0-100%). Custom firmware will handle finer granularity (0.0 - 100.0%)
int blinds_status_bits;
float blinds_voltage;
float blinds_speed;  // Speed in rpm. Custom FW will report with finer granularity

blinds_direction_t blinds_direction = DIRECTION_STOPPED;		// current direction the motor is moving

float blinds_revs_left; 	// When moving 
float blinds_target_position = -1;	// target position (in motor position units e.g. between 0% and 100%)

bool blinds_force_small_steps = false;	// use quieter small movements
bool blinds_override_limits = false;	// use movement commands overriding the upper/lower limits

float blinds_revs;			// Revolutions since last stewise move command. Just for debugging purposes.. 

/*
 Keep track how much blinds are past the maximum limit (units are revolutions. 
 This is accurate since it is updated with discrete stepwise motor movements of which we know the actual revolutions
*/
float blinds_extra_revs = 0; 

unsigned long last_poll_timestamp;
unsigned long last_cmd_timestamp;
unsigned long last_move_cmd_timestamp;
unsigned long next_move_cmd_timestamp;
unsigned long last_status_timestamps[MAX_STATUS_REGISTERS];

/*
 When using default firmware, the motor acknowledges and reports the target position with granularity of only 1% (integer units). 
 Therefore after reaching the integer position,  we go (if needed) the rest of the way (fractional part) 
 with discrete steps (90, 17.14 or 6.316 degrees). The finetune_threshold is compared to
 abs(reported_motor_position - target_position) and  uses same scale as motor position (0-100% degrees). 
 */
float finetune_threshold = 1.0;

static const char * direction2txt[3] = { "up", "stopped", "down" };

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

static const char cmd_reset[2] = { 0xfa, 0x00 };

// commands supported by custom firmware
static const char cmd_set_max_length[2] = { 0xfa, 0xee };
static const char cmd_set_full_length[2] = { 0xfa, 0xcc };

static const char cmd_ext_version[2] = { 0xcc, 0xdc };
static const char cmd_ext_status[2] = { 0xcc, 0xde };
static const char cmd_ext_limits[2] = { 0xcc, 0xdf };
static const char cmd_ext_set_speed[2] = { 0x20, 0x00 };            // second byte is the speed
static const char cmd_ext_set_default_speed[2] = { 0x30, 0x00 };    // second byte is the default speed
static const char cmd_ext_set_minimum_voltage[2] = { 0x40, 0x00 };  // second byte is minimum operating voltage (0x00 = no minimum)

static const char cmd_ext_go_to[2] = { 0x10, 0x00 }; // target position is the lower 4 bits of the 1st byte + 2nd byte (12 bits of granularity)


char * blinds_get_version() {
    return version;
}

bool blinds_is_custom_firmware() {
    return custom_fw;
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

int blinds_generic_msg( blinds_cmd_type cmd ) {
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


void blinds_process_status_reg_1( int status_bits, int voltage, int speed, int pos ) {

    blinds_status_bits = status_bits;
    blinds_voltage = voltage/30;

    blinds_speed = speed;
    if (blinds_motor_pos != pos)
    	blinds_motor_position_updated(pos);
    blinds_motor_pos = pos;

    if (blinds_target_position != -1) {
    	if (blinds_status == BLINDS_MOVING) { 
            if (!custom_fw) {
                int target_pos_integer = blinds_target_position;
                if (blinds_target_position - (float)target_pos_integer > 0.01) {
                    ESP_LOGW(TAG, "Reaching target! Moving additional %.1f%% by small steps.", blinds_target_position - (float)target_pos_integer );
    	        	blinds_status = BLINDS_MOVING_STEPS; // move by little steps the rest of the way
    	        	blinds_override_limits = true; // use forceable movement commands to avoid accidentally tripping continous movement when using cmd_up_17 at pos 0x00..
    				next_move_cmd_timestamp = iot_timestamp() + 1000;
    		    }
            }
		} else if (blinds_status == BLINDS_MOVING_STEPS) { 
	    	if ( ( (blinds_direction == DIRECTION_UP) && (blinds_motor_pos <= blinds_target_position ) ) ||
	    	  	 ( (blinds_direction == DIRECTION_DOWN) && (blinds_motor_pos >= blinds_target_position ) ) ) {
			    ESP_LOGW(TAG, "Reached target! Stopped after %.1f stepwise revolutionss..", blinds_revs);
	        	blinds_status = BLINDS_STOPPED;
		        blinds_target_position = -1;
		        blinds_revs = 0;
	    	}
	    }
	}
    
    if ( (blinds_status == BLINDS_UNKNOWN) && (blinds_speed>0) ) {
        ESP_LOGW(TAG, "Blinds automatically resetting...");
        blinds_status = BLINDS_RESETTING;
        polling_interval = DEFAULT_POLLING_INTERVAL;
    } else if ( (blinds_status == BLINDS_UNKNOWN) && (blinds_speed==0) ) {
        ESP_LOGI(TAG, "Blinds are stopped...");
        blinds_status = BLINDS_STOPPED;
        blinds_direction = 0;
    } else if (blinds_status == BLINDS_RESETTING) {
        if ( (blinds_speed == 0) && (blinds_motor_pos==0) ) {
            ESP_LOGI(TAG, "Blinds reset successfully..");
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

    if (blinds_motor_pos < 100) {
        blinds_extra_revs = 0;
    }

	last_status_timestamps[STATUS_REG_1] = iot_timestamp();

    ESP_LOGI(TAG,"STATUS:%d, st_bits 0x%x, %1.1fV, SPD %.1f RPM, POS %1.f, STEPW_REVS %.2f XTRA_REVS %.2f TARGET %.2f", 
        blinds_status, blinds_status_bits, blinds_voltage, blinds_speed, blinds_motor_pos, 
        blinds_revs, blinds_extra_revs, blinds_target_position );
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
    else if (status_reg == EXT_VERSION_REG)
        blinds_send_cmd( cmd_ext_version );
	else if (status_reg == EXT_STATUS_REG)
	    blinds_send_cmd( cmd_ext_status );
	else if (status_reg == EXT_LIMIT_STATUS_REG)
	    blinds_send_cmd( cmd_ext_limits );
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
			next_move_cmd_timestamp = iot_timestamp() + 500 + revs*3000;

			blinds_task_read_status_reg(STATUS_REG_1);

	        ESP_LOGI(TAG,"Stepwise movement of %.2f degrees", revs*360);

	        if (blinds_motor_pos == 0x64)
	            blinds_extra_revs += blinds_direction*revs;
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
	ESP_LOGI(__func__,"Move: dir %d - %.2f revs, target %.2f, force_small_steps %d, force_mov %d", direction, revs, target_position, force_small_steps, override_limits);
    if ( (revs>0) || force_small_steps || override_limits ||
    	( (target_position != -1) && (abs(blinds_motor_pos-target_position) < finetune_threshold) ) ) {
    	// Here we use smaller steps because of the following reasons:
    	//	- we want to turn specific amount of revolutions
    	//	- we want quieter operation (handy with original fw)
    	//	- we need to move outside lower limit (force-flag)
    	//	- we want to finetune position when using original firmware
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
    		if (custom_fw) {
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
	            blinds_send_cmd( cmd_down );            
	            blinds_direction = DIRECTION_DOWN;
	            blinds_target_position = 100;
	        }
	    }
        polling_interval = DEFAULT_POLLING_INTERVAL;
        blinds_status = BLINDS_MOVING;
        blinds_force_small_steps = false;
        blinds_override_limits = false;
        last_move_cmd_timestamp = iot_timestamp();
    }
    return 1;
}


int blinds_move( blinds_direction_t direction, float revs, bool force_small_steps, bool override_limits ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_move;
    	msg.direction = direction;
    	msg.revs = revs;
    	msg.force_small_steps = force_small_steps;
    	msg.override_limits = override_limits;
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
    	msg.position = position;
    	msg.force_small_steps = force_small_steps;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;
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
	return blinds_generic_msg(blinds_cmd_stop);
}


int blinds_task_reset() {
    ESP_LOGI(TAG,"Resetting position and maximum blind length..");
    blinds_send_cmd( cmd_reset );
    // delay until motor has settled
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    blinds_task_read_status_reg_blocking(STATUS_REG_1, 500);
    if (blinds_motor_pos == 0x32) {
        ESP_LOGI(TAG,"Blinds have been reset. Winding up until hard stop..");
        blinds_task_move(DIRECTION_UP, -1, -1, false, false);
        blinds_status = BLINDS_RESETTING;
    } else {
        ESP_LOGE(TAG,"Blinds have NOT been reset (position %.2f != 50!)", blinds_motor_pos);
        blinds_status = BLINDS_STOPPED;
    }
    return 1;
}

int blinds_reset() {
	return blinds_generic_msg(blinds_cmd_reset);
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
    return 1;
}

int blinds_set_max_length() {
	return blinds_generic_msg(blinds_cmd_set_max_length);
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
    return 1;
}

int blinds_set_full_length() {
	return blinds_generic_msg(blinds_cmd_set_full_length);
}

int blinds_read_status_reg( status_register_t status_reg ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_status;
    	msg.position = status_reg;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
        }
    }
    return 1;
}

int blinds_read_ext_status_reg() {
	return blinds_generic_msg(blinds_cmd_ext_status);
}

int blinds_send_raw( uint8_t cmd_byte1, uint8_t cmd_byte2 ) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_raw;
    	msg.cmd_byte1 = cmd_byte1;
    	msg.cmd_byte2 = cmd_byte2;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;	
}

int blinds_task_set_speed(float speed) {
	char cmd[2];
	if (custom_fw) {
		cmd[0] = cmd_ext_set_speed[0];
		cmd[1] = (uint8_t)speed;
        blinds_send_cmd( cmd );	
    }
    return 1;
}

int blinds_set_speed(float speed) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
    	msg.cmd = blinds_cmd_set_speed;
    	msg.speed = speed;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;	
}

int blinds_task_set_default_speed(float speed) {
    char cmd[2];
    if (custom_fw) {
        cmd[0] = cmd_ext_set_default_speed[0];
        cmd[1] = (uint8_t)speed;
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_default_speed(float speed) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
        msg.cmd = blinds_cmd_set_default_speed;
        msg.speed = speed;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;  
}

int blinds_task_set_minimum_voltage(float voltage) {
    char cmd[2];
    if (custom_fw) {
        cmd[0] = cmd_ext_set_minimum_voltage[0];
        cmd[1] = (uint8_t)(voltage*16);
        blinds_send_cmd( cmd ); 
    }
    return 1;
}

int blinds_set_minimum_voltage(float voltage) {
    blinds_msg msg;
    if (blinds_queue != NULL) {
        msg.cmd = blinds_cmd_set_minimum_voltage;
        msg.position = voltage;
        if( xQueueSend( blinds_queue, ( void * ) &msg, ( TickType_t ) 10) != pdPASS ) {
            ESP_LOGE(TAG,"error queueing blinds msg!");
            return 0;
            }
     }
     return 1;  
}

float blinds_get_speed() {
	return blinds_speed;
}

float blinds_get_pos() {
	return blinds_motor_pos;
}

blinds_status_t blinds_get_status() {
    return blinds_status;
}

blinds_direction_t blinds_get_direction() {
    return blinds_direction;
}




extern int uart_read( uint8_t * rx_buffer, int bytes, int timeout );


void blinds_process_limit_status_reg( int resetting, int max_length, int full_length ) {
	ESP_LOGI(UART_TAG,"EXT_LIMIT_STAT: resetting: %d, max_length %d, full_length %d", resetting, max_length, full_length );
	last_status_timestamps[EXT_LIMIT_STATUS_REG] = iot_timestamp();
}

void blinds_process_ext_status_reg( int status, int current, float position) {
	ESP_LOGI(UART_TAG,"EXT_STAT: status: %d, current %d, position %.2f", status, current, position);
	last_status_timestamps[EXT_STATUS_REG] = iot_timestamp();
}

void blinds_process_ext_version_reg( int version_major, int version_minor, int voltage_check ) {
    ESP_LOGI(UART_TAG,"VERSION: version %d.%d. Minimum voltage: %.2f", version_major, version_minor, (float)voltage_check/16);
    snprintf(version, MAX_VERSION_LENGTH, "%d.%d", version_major, version_minor);
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
  For detailed information about communication protocol, see fyrtyr motor board firmware documentation 
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
                        blinds_process_ext_version_reg( data[3], data[4], data[5] );
                        packet_state = PACKET_VALID;
                    } else {
                        packet_state = PACKET_INVALID;
                    }
                }

		    } else if ( (data[0]==0x00) && (data[1]==0xff) && (data[2]==0xda) ) {
		        // EXTENDED STATUS
		    	if (totRxBytes != 8) {
		    		expectedBytes = 8;
		    	} else {
			        checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
			        if (checksum == data[7]) {
			        	blinds_process_ext_status_reg( data[3], data[4], data[5]+ (float)data[6]/256 );
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
			            blinds_process_limit_status_reg( data[3], data[4]*256 + data[5], data[6]*256 + data[7] );
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
    	        			ESP_LOGI(TAG,"Move (direction %s, revs %.2f, force_small_steps-flag %d, override_limits: %d", direction2txt[msg.direction+1], msg.revs, msg.force_small_steps, msg.override_limits);
    	        			blinds_task_move(msg.direction, msg.revs, -1, msg.force_small_steps, msg.override_limits);
	            		}
        	    		break;
            		case blinds_cmd_go_to: {
    	        			ESP_LOGI(TAG,"Move to position (pos %.2f, force_small_steps-flag: %d)", msg.position, msg.force_small_steps);
    	        			blinds_task_go_to(msg.position, msg.force_small_steps);
	            		}
        	    		break;
            		case blinds_cmd_set_speed: {
            				if (custom_fw) {
	    	        			ESP_LOGI(TAG,"Setting speed to %.1f rpm", msg.speed);
    		        			blinds_task_set_speed(msg.speed);
            				} else {
	    	        			ESP_LOGE(TAG,"Setting speed not supported on original firmware!");            					
            				}
	            		}
        	    		break;
                    case blinds_cmd_set_default_speed: {
                            if (custom_fw) {
                                ESP_LOGI(TAG,"Setting default speed to %.1f rpm", msg.speed);
                                blinds_task_set_default_speed(msg.speed);
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
            		case blinds_cmd_reset: {
           	 				ESP_LOGI(TAG,"Reset");
           	 				blinds_task_reset();
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
            		case blinds_cmd_status: {
            				ESP_LOGI(TAG,"Read status reg %d", (int)msg.position);
	            			blinds_task_read_status_reg(msg.position);
            			}
            			break;
            		case blinds_cmd_raw: {
    	        			ESP_LOGI(TAG,"Send raw command (0x%x 0x%x)", msg.cmd_byte1, msg.cmd_byte2);
    	        			blinds_send_cmd_bytes(msg.cmd_byte1, msg.cmd_byte2);
	            		}
        	    		break;
                    case blinds_cmd_set_minimum_voltage: {
                            if (custom_fw) {
                                ESP_LOGI(TAG,"Setting minimum voltage to %.1f rpm", msg.position);
                                blinds_task_set_minimum_voltage(msg.position);
                            } else {
                                ESP_LOGE(TAG,"Setting minimum voltage not supported on original firmware!");                              
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
	            blinds_task_read_status_reg(STATUS_REG_1);
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

            custom_fw = true;
        } else {
            ESP_LOGI(TAG,"Original Fyrtur motor firmware detected");
        }


    } else {
        ESP_LOGE(TAG,"Fyrtur motor not connected!");
    }

}

