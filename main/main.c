#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "modules/utils.c"
#include "modules/imu.h"
#include "modules/elrs.h"
#include "modules/filters.h"

#include "iot_button.h"

#include "esp_timer.h"
#include "esp_log.h"

#define TAG "main"

#define DRONE_COUNT 2

// ELRS
#define INTERVALMS 3 * 1000

#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

#define ARMING_CHANNEL AUX1
#define CAMSWITCH_CHANNEL AUX2
#define FAILSAFE_CHANNEL AUX4
#define MECHANISM_CHANNEL AUX3

// Offsets
#define ROLL_CENTER 0
#define PITCH_CENTER 0
#define THROTTLE_CENTER 0
#define YAW_CENTER 0

// Buttons
#define RIGHT_POINT GPIO_NUM_27
#define RIGHT_MIDDLE GPIO_NUM_13
#define RIGHT_RING GPIO_NUM_14
#define LEFT_POINT GPIO_NUM_35
#define LEFT_MIDDLE GPIO_NUM_32
#define LEFT_RING GPIO_NUM_33

button_handle_t arming_button;
button_handle_t turn180_button;
button_handle_t switch_id_button;
button_handle_t mechanism_btn;
button_handle_t failsafe_btn;
button_handle_t recalibrate_btn;

// Remote variables
int8_t current_id = 1;
uint16_t channels[16] = { 0 };

struct full_imu_data left_imu_data;
struct full_imu_data right_imu_data;

struct imu_data left_imu_offset = { 0 };
struct imu_data right_imu_offset = { 0 };
struct imu_data left_imu_center = { THROTTLE_CENTER, YAW_CENTER, 0 };
struct imu_data right_imu_center = { PITCH_CENTER, ROLL_CENTER, 0 };

int left_error = 0;
int right_error = 0;
bool left_calibrated = 0;
bool right_calibrated = 0;
bool should_transmit = 0;

bool should_switch = 1;
bool yaw_lock = 0;

crsf_data_t crsf_data = {0};
pid_controller_t yaw_pid;

// Callbacks
void toggle_channel(void *arg, void *data) {
    crsf_channels_type channel = (crsf_channels_type)data;
    channels[channel] = (channels[channel] <= 1000) * MAX_CHANNEL_VALUE;
}

void arm_cb() {
    // Disable mechanism on armed and disarmed
    channels[MECHANISM_CHANNEL] = 0;
    toggle_channel(NULL, (void*)ARMING_CHANNEL);
}

void switch_cb() {
    should_switch = 1;
    current_id++;
    current_id %= DRONE_COUNT;
    ESP_LOGI("gpio", "Switch id to %d", current_id);
}

void reset_cb() {
    left_imu_offset = imu_substract_return(left_imu_data.processed, left_imu_center);
    right_imu_offset = imu_substract_return(right_imu_data.processed, right_imu_center);
    ESP_LOGI("gpio", "Remote Recalibrated");
}

void turn180_cb(){
    yaw_pid.is_active = !yaw_pid.is_active;
    if(yaw_pid.is_active){
        yaw_pid.target_angle = normalize_angle(crsf_data.attitude.yaw + 180);
    }
    ESP_LOGI("gpio", "turn is %d", yaw_pid.is_active);
}

// Timer
static void timer_callback(void *arg) {
    should_transmit = 1;
}

void timer_init() {
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "timer_callback"
    };

    esp_timer_handle_t timer_periodic;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_periodic));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_periodic, INTERVALMS));
}

// Initializers
void gpio_init() {
    // init buttons
    arming_button = init_btn(RIGHT_RING); 
    turn180_button = init_btn(RIGHT_MIDDLE);  
    switch_id_button = init_btn(LEFT_RING);
    mechanism_btn = init_btn(RIGHT_POINT);
    failsafe_btn = init_btn(LEFT_MIDDLE);
    // recalibrate_btn = init_btn(LEFT_POINT);
    
    // register callbacks
    iot_button_register_cb(arming_button, BUTTON_SINGLE_CLICK, arm_cb, NULL);
    iot_button_register_cb(turn180_button, BUTTON_SINGLE_CLICK, turn180_cb, NULL);
    iot_button_register_cb(switch_id_button, BUTTON_SINGLE_CLICK, switch_cb, NULL);
    iot_button_register_cb(mechanism_btn, BUTTON_SINGLE_CLICK, toggle_channel, (void *)MECHANISM_CHANNEL);
    iot_button_register_cb(failsafe_btn, BUTTON_SINGLE_CLICK, toggle_channel, (void *)FAILSAFE_CHANNEL);
    // iot_button_register_cb(recalibrate_btn, BUTTON_SINGLE_CLICK, reset_cb, NULL);
}

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600, //912600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_mode(UART_NUM, UART_MODE_UART);
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV || UART_SIGNAL_RXD_INV);
}

void i2c_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// Tasks
void left_imu_task() {
    init_yaw_pid(&yaw_pid);

    left_imu_data = create_full_imu_data();
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    // Calibration
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &left_imu_data.gyro, &left_imu_data.acce);

        imu_add(&left_imu_data.offset, left_imu_data.gyro);
    }
    imu_divide_single(&left_imu_data.offset, 2000);
    left_calibrated = 1;
    ESP_LOGI(TAG, "Left Gyro calibration done!");

    struct mahony_filter mahony = create_mahony_filter(NULL);

    while (true) {
        if(!imu_read(imu, &left_imu_data)) {
            left_error++;
            channels[YAW] = MID_CHANNEL_VALUE;

            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        
        apply_mahony_filter(&mahony, &left_imu_data.gyro, &left_imu_data.acce,
            left_imu_data.delta_t, left_imu_data.q);
        imu_process(&left_imu_data);
    
        imu_add(&left_imu_data.processed, left_imu_offset);
        applyDeadzone(&left_imu_data.processed.y, 0, 10);

        channels[THROTTLE] = mapValue(left_imu_data.processed.x - left_imu_offset.x, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[YAW] = mapValue(left_imu_data.processed.y - left_imu_offset.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        
        // Yaw lock (when armed only)
        if(yaw_lock && channels[ARMING_CHANNEL] == MAX_CHANNEL_VALUE) channels[YAW] = MID_CHANNEL_VALUE;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void right_imu_task() {
    right_imu_data = create_full_imu_data();
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

    // Calibration
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &right_imu_data.gyro, &right_imu_data.acce);

        imu_add(&right_imu_data.offset, right_imu_data.gyro);
    }
    imu_divide_single(&right_imu_data.offset, 2000);
    ESP_LOGI(TAG, "Right Gyro calibration done!");
    right_calibrated = 1;

    struct mahony_filter mahony = create_mahony_filter(NULL);


    while (true) {
        if(!imu_read(imu, &right_imu_data)) {
            right_error++;
            channels[ROLL] = MID_CHANNEL_VALUE;
            channels[PITCH] = MID_CHANNEL_VALUE;

            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        apply_mahony_filter(&mahony, &right_imu_data.gyro, &right_imu_data.acce,
            right_imu_data.delta_t, right_imu_data.q);
        imu_process(&right_imu_data);

        applyDeadzone(&right_imu_data.processed.y, right_imu_offset.y, 10);
        applyDeadzone(&right_imu_data.processed.x, right_imu_offset.x, 10);

        channels[ROLL] = mapValue(right_imu_data.processed.y - right_imu_offset.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[PITCH] = mapValue(right_imu_data.processed.x - right_imu_offset.x, -45, 45, 0, MAX_CHANNEL_VALUE);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void elrs_reader_task(){
    uint8_t buffer[MAX_PACKET_LENGTH] = {0};
    size_t len = 0;

    while (1)
    {
        len = uart_read_bytes(UART_NUM, buffer, MAX_PACKET_LENGTH, pdMS_TO_TICKS(15));
        process_crsf_data(buffer, &len, &crsf_data);
    }
}

void elrs_writer_task(void *pvParameters) {
    uint8_t packet[MAX_PACKET_LENGTH] = { 0 };

    while (true) {
        if (should_switch) {
            create_model_switch_packet(current_id, packet);
            elrs_send_data(UART_NUM, packet, MODEL_SWITCH_PACKET_LENGTH);
            should_switch = 0;
        } else if (should_transmit && left_calibrated && right_calibrated) {
            should_transmit = 0;
            update_yaw_pid(channels, &yaw_pid, crsf_data.attitude.yaw, xTaskGetTickCount() * portTICK_PERIOD_MS);
            create_crsf_channels_packet(channels, packet);
            elrs_send_data(UART_NUM, packet, CHANNEL_PACKET_LENGTH);
            // ESP_LOGI("channel", "a%dfs%did%dmech%dturn%dler%drer%d", channels[ARMING_CHANNEL], channels[FAILSAFE_CHANNEL], current_mechanism, current_id, yaw_pid.is_active, left_error, right_error);
            ESP_LOGI("channel", "id%dturn%dyaw%.2f", current_id, yaw_pid.is_active, crsf_data.attitude.yaw);
            
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}



void app_main(void) {
    gpio_init();
    uart_init();
    i2c_init();
    timer_init();

    xTaskCreatePinnedToCore(left_imu_task, "left_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(right_imu_task, "right_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(elrs_reader_task, "elrs_reader", 4096, NULL, tskIDLE_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(elrs_writer_task, "elrs_writer", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 1);
}  