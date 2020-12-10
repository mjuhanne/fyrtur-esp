#include "sdkconfig.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#define ESP32
#endif

typedef enum motor_firmware_status {
	MOTOR_NOT_DETECTED = 0,
	ORIGINAL_FW,
	CUSTOM_FW
} motor_firmware_status_t;

typedef enum blinds_status {
	BLINDS_UNKNOWN = 0,
	BLINDS_STOPPED,
	BLINDS_STOPPING,
	BLINDS_MOVING,
	BLINDS_MOVING_STEPS,
	BLINDS_CALIBRATING
} blinds_status_t;

typedef enum blinds_direction_ {
	DIRECTION_UP = -1,
	DIRECTION_STOPPED = 0,
	DIRECTION_DOWN = 1
} blinds_direction_t;

typedef enum blinds_cmd_t {
	blinds_cmd_move = 0,
	blinds_cmd_go_to,
	blinds_cmd_set_speed,
	blinds_cmd_set_default_speed,
	blinds_cmd_stop,
	blinds_cmd_reset_max_length,
	blinds_cmd_set_max_length,
	blinds_cmd_set_full_length,
	blinds_cmd_status,
	blinds_cmd_send_raw,
	blinds_cmd_ext_status,
	blinds_cmd_set_minimum_voltage,
	blinds_cmd_set_location,
	blinds_cmd_go_to_location,
	blinds_cmd_set_auto_cal,
	blinds_cmd_reset_full_length,
	blinds_cmd_set_orientation,
	blinds_cmd_set_stall_detection_timeout,
	blinds_cmd_set_max_motor_current,
	blinds_cmd_toggle_orientation,
	blinds_cmd_reset_orientation
} blinds_cmd_t;

typedef enum blinds_variable_t {
	BLINDS_POSITION = 0,
	BLINDS_SPEED,
	BLINDS_VOLTAGE,
	BLINDS_MOTOR_CURRENT,
	BLINDS_LOCATION,
	BLINDS_TARGET_LOCATION,
	BLINDS_MOTOR_STATUS,
	BLINDS_MAX_LEN,
	BLINDS_FULL_LEN,
	BLINDS_CALIBRATING_STATUS,
	BLINDS_STATUS,
	BLINDS_DIRECTION,
	BLINDS_TARGET_POSITION,
	BLINDS_ORIENTATION
} blinds_variable_t;

typedef enum blinds_orientation_t {
	ORIENTATION_NORMAL = 0,
	ORIENTATION_REVERSED = 1
} blinds_orientation_t;

typedef struct blinds_msg {
    blinds_cmd_t cmd;
    //blinds_direction_t direction;
    //float position, revs;
    int int_param_1, int_param_2, int_param_3;
    float float_param_1;
    //bool override_limits;
    //bool force_small_steps;
    //uint8_t cmd_byte1, cmd_byte2;
} blinds_msg;

typedef enum status_register_t {
// These are the status registers available on original firmware
    STATUS_REG_1 = 0,
    STATUS_REG_2 = 1,
    STATUS_REG_3 = 2,
    STATUS_REG_4 = 3,
// Extended status registers to be used with custom firmware
    EXT_LOCATION_REG = 4,
    EXT_VERSION_REG = 5,
    EXT_STATUS_REG = 6,
    EXT_LIMIT_STATUS_REG = 7,
    EXT_DEBUG_REG = 8,
    EXT_SENSOR_DEBUG_REG = 9,
    EXT_TUNING_PARAMS_REG = 10,
    MAX_STATUS_REGISTERS = 11,
} status_register_t;

void blinds_init();

// -- These commands work on both original and custom firmware
//int blinds_send_cmd_bytes( uint8_t cmd_byte1, uint8_t cmd_byte2 );
//int blinds_send_cmd( const char * cmd_bytes );
int blinds_send_raw( uint8_t cmd_byte1, uint8_t cmd_byte2 );

int blinds_move( blinds_direction_t direction, float revolutions, bool force_small_steps, bool force );
int blinds_stop();

int blinds_go_to( float position, bool silent );

int blinds_reset_max_length(); // will reset max curtain length to full length and calibrate curtain position by rolling it up
int blinds_set_max_length();	// sets the max curtain length to the current position
int blinds_set_full_length();	// sets the max curtain length to the current position

int blinds_read_status_reg(status_register_t status_reg);

float blinds_get_position();
float blinds_get_voltage();
int blinds_get_speed();

blinds_status_t blinds_get_status();
const char * blinds_get_status_str();
blinds_direction_t blinds_get_direction();
const char * blinds_get_direction_str();

motor_firmware_status_t blinds_get_firmware_status();

// These commands work only on custom firmware
int blinds_set_speed(int speed);
int blinds_set_default_speed(int speed);
int blinds_set_minimum_voltage(float voltage);
int blinds_set_auto_cal(bool enabled);
int blinds_set_orientation( blinds_orientation_t orientation );
int blinds_toggle_orientation();
int blinds_reset_orientation();
int blinds_go_to_location( int location );
int blinds_get_target_position();
int blinds_get_location();
int blinds_get_target_location();
int blinds_get_full_length();
int blinds_get_max_length();
int blinds_get_target_speed();
int blinds_get_calibration_status();
int blinds_get_orientation();
int blinds_get_current();
const char * blinds_get_motor_status_str();
char * blinds_get_version();

int blinds_set_location(int location);

int blinds_reset_full_length();	// will reset FULL curtain length to original factory length (13 revolutions + 265 degrees) and calibrate curtain position by rolling it up

void blinds_set_diagnostics( bool diagnostics );

// Callback
extern void blinds_variable_updated( blinds_variable_t variable );
