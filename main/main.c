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

attitude_data_t  current_attiitude = {0};

// ELRS UART
#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

//TIMER
#define INTERVALMS 3 * 1000 // in microseconds

pid_controller_t yaw_pid;


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

int left_error = 0;
int right_error = 0;

struct imu_data left_imu_offset = { 0 };
struct imu_data right_imu_offset = { 0 };

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

void turn_command_cb(){
    yaw_pid.is_active = !yaw_pid.is_active;
    ESP_LOGI("gpio", " turn is %d", yaw_pid.is_active);

}

void gpio_init() {
    ESP_ERROR_CHECK(button_init(&arming_button, /* Button instance */
        GPIO_NUM_27,                       /* Button GPIO number */
        BUTTON_EDGE_FALLING,
        tskIDLE_PRIORITY + 10,            /* Button FreeRTOS task priority */
        configMINIMAL_STACK_SIZE * 4));   /* Button FreeRTOS task stack size */
    ESP_ERROR_CHECK(button_init(&mekanisme_button, /* Button instance */
        GPIO_NUM_13,                       /* Button GPIO number */
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
    button_add_cb(&arming_button, BUTTON_CLICK_SINGLE, AUX1_cb, NULL);

    // mekanisme_button
    // button_add_cb(&mekanisme_button, BUTTON_CLICK_SINGLE, AUX2_cb, NULL);
    button_add_cb(&mekanisme_button, BUTTON_CLICK_MEDIUM, turn_command_cb, NULL);
    button_add_cb(&mekanisme_button, BUTTON_CLICK_LONG, turn_command_cb, NULL);


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
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV || UART_SIGNAL_RXD_INV);
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


    init_yaw_pid(&yaw_pid);

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
        if(!imu_read(imu, &left_imu_data)) {
            left_error++;
            channels[YAW] = MAX_CHANNEL_VALUE / 2;

            vTaskDelay(2 / portTICK_PERIOD_MS);
            continue;
        }
        
        // Do filtering
        apply_mahony_filter(&mahony, &left_imu_data.gyro, &left_imu_data.acce,
            left_imu_data.delta_t, left_imu_data.q);
        imu_process(&left_imu_data);
    
        imu_add(&left_imu_data.processed, left_imu_offset);
        applyDeadzone(&left_imu_data.processed.y, 0, 10);


        /*Mapping degree to PWM*/
        channels[THROTTLE] = mapValue(left_imu_data.processed.x - left_imu_offset.x, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[YAW] = mapValue(left_imu_data.processed.y - left_imu_offset.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        
        if(yaw_lock && channels[AUX1] == MAX_CHANNEL_VALUE) channels[YAW] = MAX_CHANNEL_VALUE / 2;
        // printf("-");

        update_yaw_pid(channels, &yaw_pid, current_attiitude.yaw, xTaskGetTickCount() * portTICK_PERIOD_MS);
        

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
        if(!imu_read(imu, &right_imu_data)) {
            right_error++;
            channels[ROLL] = MAX_CHANNEL_VALUE / 2;
            channels[PITCH] = MAX_CHANNEL_VALUE / 2;

            vTaskDelay(2 / portTICK_PERIOD_MS);
            continue;
        }

        // Do filtering
        apply_mahony_filter(&mahony, &right_imu_data.gyro, &right_imu_data.acce,
            right_imu_data.delta_t, right_imu_data.q);
        imu_process(&right_imu_data);

        applyDeadzone(&right_imu_data.processed.y, right_imu_offset.y, 10);
        applyDeadzone(&right_imu_data.processed.x, right_imu_offset.x, 10);

        /*Mapping degree to PWM*/
        channels[ROLL] = mapValue(right_imu_data.processed.y - right_imu_offset.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[PITCH] = mapValue(right_imu_data.processed.x - right_imu_offset.x, -45, 45, 0, MAX_CHANNEL_VALUE);

        // printf("r%dp%d\n", channels[ROLL], channels[PITCH]);


        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

int read_telemetry(){
    uint8_t rx_buffer[256];
    size_t rx_buffer_len = uart_read_bytes(UART_NUM, rx_buffer, 256, 15 / portTICK_PERIOD_MS);

    // if(rx_buffer_len > 1) ESP_LOGI("uart", "length %d", rx_buffer_len);

    if (process_crsf_uart_data(rx_buffer, &rx_buffer_len, &current_attiitude)) {
        // ESP_LOGI("telemetry", "R%.2f/ P%.2f Y%.2f V%.2f", current_attiitude.roll, current_attiitude.pitch,  current_attiitude.yaw, current_attiitude.vspd);
        return 1;
    }
    return 0;
}





void elrs_task(void *pvParameters) {
    gpio_init();
    uint8_t packet[MAX_PACKET_LENGTH] = { 0 };

    while (true) {
        if (should_switch) {
            create_model_switch_packet(current_id, packet);
            elrs_send_data(UART_NUM, packet, MODEL_SWITCH_PACKET_LENGTH);
            should_switch = 0;
        } else if (should_transmit && left_calibrated && right_calibrated) {


            should_transmit = false;
            create_crsf_channels_packet(channels, packet);
            elrs_send_data(UART_NUM, packet, CHANNEL_PACKET_LENGTH);
            ESP_LOGI("channel", "r%dp%dt%dy%d arming %d, failsafe %d, id %d, left error %d, right error %d", channels[ROLL], channels[PITCH], channels[THROTTLE], channels[YAW], channels[AUX1], channels[AUX2], current_id, left_error, right_error);

            read_telemetry();
        }

        vTaskDelay(1 / portTICK_PERIOD_MS); // ojo diganti
    }
}



void app_main(void) {
    uart_init();
    i2c_init();
    timer_init();

    // switch_id();
    // switch_init();

    xTaskCreatePinnedToCore(left_imu_task, "left_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(right_imu_task, "right_imu", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(elrs_task, "send_payload", 4096, NULL, tskIDLE_PRIORITY, NULL, 1);
}  