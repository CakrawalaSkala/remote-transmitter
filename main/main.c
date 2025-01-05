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
#include "modules/button.h"

#include "esp_timer.h"
#include "esp_log.h"

#define TAG "main"

#define TOGGLE_CHANNEL_CB(channel) void channel##_cb() { channels[channel] = (channels[channel] <= 1000) * 2000; }

// static const char *TAG = "Tes";

// ELRS UART
#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

//TIMER
#define INTERVALMS 3 * 1000 // in microseconds

static bool left_calibrated = 0;
static bool right_calibrated = 0;

static void timer_callback(void *arg);
static bool should_transmit = 0;
static bool should_switch = 1;
static bool yaw_lock = 0;
int8_t current_id = 0;
// IMU
uint16_t channels[16] = { 0 };

TOGGLE_CHANNEL_CB(AUX1);
TOGGLE_CHANNEL_CB(AUX2);
TOGGLE_CHANNEL_CB(AUX3);

bool tx_in_use = 1;


button_t arming_button;
button_t mekanisme_button;
button_t switch_button;

#define ROLL_CENTER 0
#define PITCH_CENTER 0
#define THROTTLE_CENTER 0
#define YAW_CENTER 0

struct imu_data left_imu_center = { THROTTLE_CENTER, YAW_CENTER, 0 };
struct imu_data right_imu_center = { PITCH_CENTER, ROLL_CENTER, 0 };

struct full_imu_data left_imu_data;
struct full_imu_data right_imu_data;

float left_n = 0;
float right_n = 0;

struct imu_data left_imu_offset = { 0 };
struct imu_data right_imu_offset = { 0 };


// Parse attitude data from buffer
bool parse_attitude_data(uint8_t *data, attitude_data_t *attitude) {
    if (data[2] != ATTITUDE_PACKET_TYPE) {
        return false;
    }
    
    // Convert bytes to float values
    attitude->pitch = (float)(int16_t)((data[3] << 8) | data[4]) / 10000.0f;
    attitude->roll = (float)(int16_t)((data[5] << 8) | data[6]) / 10000.0f;
    attitude->yaw = (float)(int16_t)((data[7] << 8) | data[8]) / 10000.0f;
    
    return true;
}

void read_crsf_attitude(void) {
    uint8_t data[BUF_SIZE];
    attitude_data_t attitude = {0};
    
    while (1) {
        // Read data from UART
        if(tx_in_use) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Look for valid packets in the received data
            for (int i = 0; i < len - ATTITUDE_PACKET_LENGTH; i++) {
                if (parse_attitude_data(&data[i], &attitude)) {
                    ESP_LOGI(TAG, "Attitude: Pitch=%.2f Roll=%.2f Yaw=%.2f (rad)",
                            attitude.pitch, attitude.roll, attitude.yaw);
                    break;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // Add small delay to prevent CPU hogging
    }
}


void handle_read_crsf_attitude(void) {
    uint8_t data[BUF_SIZE];
    attitude_data_t attitude = {0};

        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Look for valid packets in the received data
            for (int i = 0; i < len - ATTITUDE_PACKET_LENGTH; i++) {
                printf("parsing \n");
                if (parse_attitude_data(&data[i], &attitude)) {
                    ESP_LOGI(TAG, "Attitude: Pitch=%.2f Roll=%.2f Yaw=%.2f (rad)",
                            attitude.pitch, attitude.roll, attitude.yaw);
                    break;
                }
            }
        }    
    }


void  switch_cb() {
    // yaw_lock = !yaw_lock;
    should_switch = 1;
    current_id = current_id < 2 ? current_id + 1 : 0;
    ESP_DRAM_LOGI("gpio", "current_id %d", current_id);
}

void reset_cb() {
    ESP_LOGI("gpio", "reset_cb");
    left_imu_offset = imu_substract_return(left_imu_data.processed, left_imu_center);
    right_imu_offset = imu_substract_return(right_imu_data.processed, right_imu_center);
}

void gpio_init() {
    ESP_ERROR_CHECK(button_init(&arming_button, /* Button instance */
        GPIO_NUM_13,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */
    ESP_ERROR_CHECK(button_init(&mekanisme_button, /* Button instance */
        GPIO_NUM_27,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */

    ESP_ERROR_CHECK(button_init(&switch_button, /* Button instance */
        GPIO_NUM_14,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */
    
    // arming_button
    button_add_cb(&arming_button, BUTTON_CLICK_LONG, AUX1_cb, NULL);

    button_add_cb(&arming_button, BUTTON_CLICK_MEDIUM, AUX1_cb, NULL);
    // button_add_cb(&arming_button, BUTTON_CLICK_LONG, AUX1_cb, NULL);
    // button_add_cb(&arming_button, BUTTON_CLICK_SINGLE, AUX1_cb, NULL);

    // mekanisme_button
    // button_add_cb(&mekanisme_button, BUTTON_CLICK_SINGLE, AUX2_cb, NULL);
    button_add_cb(&mekanisme_button, BUTTON_CLICK_MEDIUM, AUX2_cb, NULL);
    button_add_cb(&mekanisme_button, BUTTON_CLICK_LONG, switch_cb, NULL);


    // switch id
    button_add_cb(&switch_button, BUTTON_CLICK_LONG, switch_cb, NULL);
    button_add_cb(&switch_button, BUTTON_CLICK_MEDIUM, switch_cb, NULL);


    // reset_button
    // button_add_cb(&reset_button, BUTTON_CLICK_SINGLE, reset_cb, NULL);
    // button_add_cb(&reset_button, BUTTON_CLICK_LONG, reset_cb, NULL);
    // button_add_cb(&reset_button, BUTTON_CLICK_MEDIUM, reset_cb, NULL);
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


static void timer_callback(void *arg) {
    should_transmit = 1;
    // int64_t time_since_boot = esp_timer_get_time();
    // int64_t interval = time_since_boot - last_time;

    // // portENTER_CRITICAL_ISR(&spinlock);
    // should_transmit = interval < 4000 ? true : false;
    // // portEXIT_CRITICAL(&spinlock);

    // if (!should_transmit) ESP_LOGI(TAG, "interval is %lld us", interval);
    // last_time = time_since_boot;
}

void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600, //912600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver
    uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);
    // Set pins
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set UART mode to half-duplex
    uart_set_mode(UART_NUM, UART_MODE_UART);
    // Invert RX and TX signals
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    // UART_SIGNAL_RXD_INV
}




// IMU related task
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

void left_imu_task() {
    left_imu_data = create_full_imu_data();
    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &left_imu_data.gyro, &left_imu_data.acce);

        imu_add(&left_imu_data.offset, left_imu_data.gyro);
    }
    imu_divide_single(&left_imu_data.offset, 2000);
    left_calibrated = 1;
    ESP_LOGI(TAG, "Left Gyro calibration done!");

    struct mahony_filter mahony = create_mahony_filter(NULL);

    while (true) {
        if(!imu_read(imu, &left_imu_data)) continue;
        
        left_n++;
        // Do filtering
        apply_mahony_filter(&mahony, &left_imu_data.gyro, &left_imu_data.acce,
            left_imu_data.delta_t, left_imu_data.q);
        imu_process(&left_imu_data);
    
        imu_add(&left_imu_data.processed, left_imu_offset);
        applyDeadzone(&left_imu_data.processed.y, 0, 10);


        /*Mapping degree to PWM*/
        if (left_n > 1) {
            channels[THROTTLE] = (left_n - 1) / left_n * channels[THROTTLE] + 1 / left_n * mapValue(left_imu_data.processed.x - left_imu_offset.x, -45, 45, MAX_CHANNEL_VALUE, 0);
            channels[YAW] = (left_n - 1) / left_n * channels[YAW] + 1 / left_n * mapValue(left_imu_data.processed.y - left_imu_offset.y, -45, 45, MAX_CHANNEL_VALUE, 0);
        } else {
            channels[THROTTLE] = mapValue(left_imu_data.processed.x - left_imu_offset.x, -45, 45, MAX_CHANNEL_VALUE, 0);
            channels[YAW] = mapValue(left_imu_data.processed.y - left_imu_offset.y, -45, 45, MAX_CHANNEL_VALUE, 0);
        }
        if(yaw_lock && channels[AUX1] == MAX_CHANNEL_VALUE) channels[YAW] = MAX_CHANNEL_VALUE / 2;
        // printf("-");

        // Deadzone
        // applyDeadzone(&channels[YAW], 1500, 50);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void right_imu_task() {
    right_imu_data = create_full_imu_data();
    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &right_imu_data.gyro, &right_imu_data.acce);

        imu_add(&right_imu_data.offset, right_imu_data.gyro);
    }
    imu_divide_single(&right_imu_data.offset, 2000);
    ESP_LOGI(TAG, "Right Gyro calibration done!");
    right_calibrated = 1;

    struct mahony_filter mahony = create_mahony_filter(NULL);


    while (true) {
        if(!imu_read(imu, &right_imu_data)) continue;

        right_n++;
        // Do filtering
        apply_mahony_filter(&mahony, &right_imu_data.gyro, &right_imu_data.acce,
            right_imu_data.delta_t, right_imu_data.q);
        imu_process(&right_imu_data);

        applyDeadzone(&right_imu_data.processed.y, right_imu_offset.y, 10);
        applyDeadzone(&right_imu_data.processed.x, right_imu_offset.x, 10);

        /*Mapping degree to PWM*/
        if (right_n > 1) {
            channels[ROLL] = (right_n - 1) / right_n * channels[ROLL] + 1 / right_n * mapValue(left_imu_data.processed.y - left_imu_offset.y, -45, 45, MAX_CHANNEL_VALUE, 0);
            channels[PITCH] = (right_n - 1) / right_n * channels[PITCH] + 1 / right_n * mapValue(left_imu_data.processed.x - left_imu_offset.x, -45, 45, MAX_CHANNEL_VALUE, 0);
        } else {
            channels[ROLL] = mapValue(right_imu_data.processed.y - right_imu_offset.y, -45, 45, MAX_CHANNEL_VALUE, 0);
            channels[PITCH] = mapValue(right_imu_data.processed.x - right_imu_offset.x, -45, 45, MAX_CHANNEL_VALUE, 0);
        }
        // printf("+");

        // printf("r%dp%d\n", channels[ROLL], channels[PITCH]);


        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

void elrs_task(void *pvParameters) {
    uint8_t packet[MAX_PACKET_LENGTH] = { 0 };

    while (true) {
        if (should_switch) {
            create_model_switch_packet(current_id, packet);
            tx_in_use = 1;
            elrs_send_data(UART_NUM, packet, MODEL_SWITCH_PACKET_LENGTH);
            tx_in_use = 0;
            send_subscribe_packet();

            should_switch = 0;
        }
        if (should_transmit && left_calibrated && right_calibrated) {
            should_transmit = false;
            create_crsf_channels_packet(channels, packet);
            gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(TX_PIN, GPIO_PULLUP_ONLY);
            tx_in_use = 1;

            elrs_send_data(UART_NUM, packet, CHANNEL_PACKET_LENGTH);
            // handle_read_crsf_attitude();
            tx_in_use = 0;


            // printf("r%dp%dt%dy%d arming %d, failsafe %d, id %d\n", channels[ROLL], channels[PITCH], channels[THROTTLE], channels[YAW], channels[AUX1], channels[AUX2], current_id);
            left_n = 0;
            right_n = 0;
            gpio_set_direction(TX_PIN, GPIO_MODE_INPUT);
            gpio_set_pull_mode(TX_PIN, GPIO_FLOATING);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS); // ojo diganti

    }
}



void app_main(void) {
    uart_init();
    i2c_init();
    timer_init();
    gpio_init();
    
    // switch_id();
    // switch_init();

    xTaskCreatePinnedToCore(left_imu_task, "left_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(right_imu_task, "right_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(elrs_task, "send_payload", 4096, NULL, tskIDLE_PRIORITY, NULL, 1);
    // xTaskCreatePinnedToCore(read_crsf_attitude, "read", 4096, NULL, 5,NULL,  1);
}  