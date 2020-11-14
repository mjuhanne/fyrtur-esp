#include "sdkconfig.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#define ESP32
#endif

typedef enum blinds_status {
	BLINDS_UNKNOWN = -1,
	BLINDS_STOPPED = 0,
	BLINDS_STOPPING = 1,
	BLINDS_MOVING = 2,
	BLINDS_MOVING_STEPS = 3,
	BLINDS_RESETTING = 4
} blinds_status_t;

typedef enum blinds_direction_ {
	DIRECTION_UP = -1,
	DIRECTION_STOPPED = 0,
	DIRECTION_DOWN = 1
} blinds_direction_t;

typedef enum blinds_cmd_type {
	blinds_cmd_move = 0,
	blinds_cmd_go_to,
	blinds_cmd_set_speed,
	blinds_cmd_set_default_speed,
	blinds_cmd_stop,
	blinds_cmd_reset,
	blinds_cmd_set_max_length,
	blinds_cmd_set_full_length,
	blinds_cmd_status,
	blinds_cmd_raw,
	blinds_cmd_ext_status,
	blinds_cmd_set_minimum_voltage,
	blinds_cmd_set_location
} blinds_cmd_type;

typedef struct blinds_msg {
    blinds_cmd_type cmd;
    blinds_direction_t direction;
    float position, speed, revs;
    bool override_limits;
    bool force_small_steps;
    uint8_t cmd_byte1, cmd_byte2;
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
    MAX_STATUS_REGISTERS = 8,
} status_register_t;

void blinds_init();

int blinds_send_cmd_bytes( uint8_t cmd_byte1, uint8_t cmd_byte2 );
int blinds_send_cmd( const char * cmd_bytes );

int blinds_move( blinds_direction_t direction, float revolutions, bool force_small_steps, bool force );
int blinds_go_to( float position, bool silent );
int blinds_send_raw( uint8_t cmd_byte1, uint8_t cmd_byte2 );

int blinds_read_status_reg(status_register_t status_reg);
int blinds_set_speed(float speed);
int blinds_set_default_speed(float speed);
int blinds_set_minimum_voltage(float voltage);
int blinds_get_speed();
float blinds_get_pos();
bool blinds_is_custom_firmware();
char * blinds_get_version();

blinds_status_t blinds_get_status();
blinds_direction_t blinds_get_direction();

int blinds_set_location(int location);

int blinds_stop();
int blinds_reset(); 
int blinds_set_max_length();
int blinds_set_full_length();

// Callback
extern void blinds_motor_position_updated( float position );
