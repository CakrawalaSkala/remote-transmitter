
#include "driver/gpio.h"

#define MAX_CHANNEL_VALUE 1984
#define MIN_CHANNEL_VALUE 0

#include <math.h>

#define MAX_PACKET_LENGTH 26
#define CHANNEL_PACKET_LENGTH 26
#define MODEL_SWITCH_PACKET_LENGTH 10

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define RAD_TO_DEG 57.295779513f  // 180/π
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    uint32_t last_time;
    
    // New fields for event-based control
    bool is_active;         // Whether PID is currently controlling
    float target_angle;     // Stored target angle
    float start_angle;      // Starting angle when command was issued
    float angle_threshold;  // How close we need to be to target to consider it "done"
} pid_controller_t;

void start_yaw_movement(pid_controller_t *pid, float current_angle, float target_angle);
void init_yaw_pid(pid_controller_t *pid);
float normalize_angle(float angle);
float get_angle_error(float target, float current) ;

void update_yaw_pid(uint16_t *channels, pid_controller_t *pid,  float current_angle, uint32_t current_time);

#define RC_MIN 988
#define RC_MID 1500
#define RC_MAX 2012
#define RC_RANGE (RC_MAX - RC_MIN)


typedef struct {
    float pitch;
    float roll;
    float yaw;
    float vspd;
} attitude_data_t;

 bool parse_attitude(const uint8_t *data, attitude_data_t *attitude) ;
 bool crsf_validate_frame(const uint8_t *frame, uint8_t len);
 uint8_t crc8_data(const uint8_t *data, uint8_t len);
 uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
bool process_crsf_uart_data(uint8_t *input_buffer, size_t *input_len, attitude_data_t *attitude);


#define CRSF_SYNC_BYTE 0xC8
#define CRSF_ATTITUDE_PACKET 0x1E
#define CRSF_VARIO_PACKET 0x07


typedef enum {
    DEVICE_ADDRESS_BROADCAST = 0x00,
    DEVICE_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    DEVICE_ADDRESS_REMOTE_CONTROL = 0xEA,
    DEVICE_ADDRESS_TX_MODULE = 0xEE,
} crsf_device_address_t;

typedef enum {
    FRAME_TYPE_RC_CHANNELS = 0x16,
    FRAME_TYPE_PING = 0x28,
    FRAME_TYPE_DEVICE_INFO = 0x29,
    FRAME_TYPE_DIRECT_COMMANDS = 0x32,
} crsf_frame_type_t;

typedef enum {
    COMMAND_CROSSFIRE = 0x10,
    COMMAND_REMOTE_RELATED = 0x3A,
} crsf_commands_t;

typedef enum {
    CROSSFIRE_COMMAND_RECEIVER_BIND = 0x01,
    CROSSFIRE_COMMAND_CANCEL_BIND = 0x02,
    CROSSFIRE_COMMAND_SET_BIND_ID = 0x03,
    CROSSFIRE_COMMAND_MODEL_SELECT = 0x05,
} crsf_crossfire_commands_t;

typedef enum {
    REMOTE_RELATED_COMMANDS_TIMING_CORRECTION = 0x10,
} crsf_remote_related_commands_t;

typedef enum {
    ROLL = 0,
    PITCH = 1,
    THROTTLE = 2,
    YAW = 3,
    AUX1 = 4,
    AUX2 = 5,
    AUX3 = 6,
    AUX4 = 7,
    AUX5 = 8,
    AUX6 = 9,
    AUX7 = 10,
    AUX8 = 11,
    AUX9 = 12,
    AUX10 = 13,
    AUX11 = 14,
    AUX12 = 15
} crsf_channels_type;

uint8_t get_crc8(uint8_t *buf, size_t size, uint8_t poly);
void elrs_send_data(const int port, const uint8_t *data, size_t len);

void create_crsf_channels_packet(uint16_t *channels, uint8_t *packet);
void create_model_switch_packet(uint8_t id, uint8_t *packet);