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

static void timer_callback(void *arg);
static bool should_transmit = 0;
static bool should_switch = 1;
int8_t current_id = 0;
// IMU
uint16_t channels[16] = { 0 };


TOGGLE_CHANNEL_CB(AUX1);
TOGGLE_CHANNEL_CB(AUX2);
TOGGLE_CHANNEL_CB(AUX3);

button_t arming_button;
button_t mekanisme_button;

void  switch_cb() {
    should_switch = 1;
    current_id = current_id < 3 ? current_id + 1 : 0;
    ESP_LOGI("gpio", "should switch %d", current_id);
}

void gpio_init() {
    ESP_ERROR_CHECK(button_init(&arming_button, /* Button instance */
        GPIO_NUM_13,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */
    ESP_ERROR_CHECK(button_init(&mekanisme_button, /* Button instance */
        GPIO_NUM_12,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */

    button_add_cb(&arming_button, BUTTON_CLICK_MEDIUM, AUX1_cb, "button arming medium");
    button_add_cb(&mekanisme_button, BUTTON_CLICK_MEDIUM, AUX2_cb, "button mekanisme medium");
    button_add_cb(&mekanisme_button, BUTTON_CLICK_LONG, switch_cb, "switch id");
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
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV);
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
    struct full_imu_data imu_data = create_full_imu_data();

    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &imu_data.gyro, &imu_data.acce);

        imu_add(&imu_data.offset, imu_data.gyro);
    }
    imu_divide_single(&imu_data.offset, 2000);
    ESP_LOGI(TAG, "Left Gyro calibration done!");

    struct kalman_filter *kalman = create_kalman_filter(NULL);

    while (true) {
        imu_read(imu, &imu_data);
        // Do filtering
        apply_kalman_filter(kalman, &imu_data.gyro, &imu_data.acce,
            imu_data.delta_t, imu_data.q);
        imu_process(&imu_data);

        /*Mapping degree to PWM*/
        channels[THROTTLE] = mapValue(imu_data.processed.x, -30, 30, 0, MAX_CHANNEL_VALUE);
        channels[YAW] = mapValue(imu_data.processed.y, -30, 30, 0, MAX_CHANNEL_VALUE);
        // printf("-");

        // Deadzone
        // applyDeadzone(&channels[YAW], 1500, 50);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void right_imu_task() {
    struct full_imu_data imu_data = create_full_imu_data();

    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &imu_data.gyro, &imu_data.acce);

        imu_add(&imu_data.offset, imu_data.gyro);
    }
    imu_divide_single(&imu_data.offset, 2000);
    ESP_LOGI(TAG, "Right Gyro calibration done!");

    struct kalman_filter *kalman = create_kalman_filter(NULL);

    while (true) {
        imu_read(imu, &imu_data);
        // Do filtering
        apply_kalman_filter(kalman, &imu_data.gyro, &imu_data.acce,
            imu_data.delta_t, imu_data.q);
        imu_process(&imu_data);

        /*Mapping degree to PWM*/
        channels[ROLL] = mapValue(imu_data.processed.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[PITCH] = mapValue(imu_data.processed.x, -45, 45, MAX_CHANNEL_VALUE, 0);
        // printf("+");

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}
void elrs_task(void *pvParameters) {
    uint8_t channel_packet[CHANNEL_PACKET_LENGTH] = { 0 };
    uint8_t model_packet[MODEL_SWITCH_PACKET_LENGTH] = { 0 };

    while (true) {
        if (should_switch) {
            create_model_switch_packet(current_id, model_packet);
            elrs_send_data(UART_NUM, model_packet, 10);
            should_switch = 0;
        }
        if (should_transmit) {
            should_transmit = false;
            create_crsf_channels_packet(channels, channel_packet);
            elrs_send_data(UART_NUM, channel_packet, CHANNEL_PACKET_LENGTH);
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
}