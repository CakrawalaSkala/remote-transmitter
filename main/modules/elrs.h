
#include "driver/gpio.h"
#include <math.h>

#define MAX_CHANNEL_VALUE 1984
#define MAX_MECHANISM_CHANNEL_VALUE 1088
#define MID_CHANNEL_VALUE MAX_CHANNEL_VALUE / 2
#define MIN_CHANNEL_VALUE 0

#define MAX_PACKET_LENGTH 26
#define CHANNEL_PACKET_LENGTH 26
#define MODEL_SWITCH_PACKET_LENGTH 10

#define RC_MIN 1200
#define RC_MID 1225
#define RC_MAX 1250
#define RC_RANGE (RC_MAX - RC_MIN)

#define RAD_TO_DEG 57.295779513f

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

typedef struct {
    float pitch;
    float roll;
    float yaw;

    float prev_yaw;
    float yaw_rate;    // Rate of change between updates
    uint32_t last_update_time;
} attitude_data_t;

// Constants for altitude calculations
#define ALT_MIN_DM 10000                     // minimum altitude in dm (-1000m)
#define ALT_THRESHOLD_DM (0x8000 - ALT_MIN_DM) // altitude precision change threshold
#define ALT_MAX_DM ((0x7ffe * 10) - 5)       // maximum altitude in dm

// Structure for barometric data
typedef struct {
    uint16_t altitude_packed;       // Altitude above start point
    int8_t vertical_speed_packed;   // Vertical speed
} baro_data_t;


typedef struct {
    attitude_data_t attitude;
    float baro_alt;
    float vspd;
} crsf_data_t;

typedef enum {
    DEVICE_ADDRESS_BROADCAST = 0x00,
    DEVICE_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    DEVICE_ADDRESS_REMOTE_CONTROL = 0xEA,
    DEVICE_ADDRESS_TX_MODULE = 0xEE,
} crsf_device_address_t;

typedef enum {
    FRAME_TYPE_VARIO = 0x07,
    FRAME_TYPE_BARO_ALTITUDE = 0x09,
    FRAME_TYPE_RC_CHANNELS = 0x16,
    FRAME_TYPE_ATTITUDE = 0x1E,
    FRAME_TYPE_PING = 0x28,
    FRAME_TYPE_DEVICE_INFO = 0x29,
    FRAME_TYPE_DIRECT_COMMANDS = 0x32,
} crsf_frame_type_t;

typedef enum {
    COMMAND_CROSSFIRE = 0x10,
    COMMAND_REMOTE_RELATED = 0x3A,
    COMMAND_SUBSCRIBE = 0x20,
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

// Utils
uint8_t get_crc8(uint8_t *buf, size_t size, uint8_t poly);
float normalize_angle(float angle);
float get_angle_error(float target, float current);

// PID
void init_yaw_pid(pid_controller_t *pid);
void start_yaw_movement(pid_controller_t *pid, float current_angle, float target_angle);
void update_yaw_pid(uint16_t *channels, pid_controller_t *pid, float current_angle, uint32_t current_time);
float get_interpolated_yaw(attitude_data_t *attitude);

// ELRS
void parse_frame(const uint8_t *data, crsf_data_t *crsf_data);
void process_crsf_data(uint8_t *input_buffer, size_t *input_len, crsf_data_t *crsf_data);
void elrs_send_data(const int port, const uint8_t *data, size_t len);

// Packet Creation
void create_crsf_channels_packet(uint16_t *channels, uint8_t *packet);
void create_model_switch_packet(uint8_t id, uint8_t *packet);

//baro telemetry

void create_subscribe_packet(uint8_t id, uint8_t *packet);


uint16_t get_altitude_packed (int32_t altitude_dm);

int32_t get_altitude_dm(uint16_t packed);