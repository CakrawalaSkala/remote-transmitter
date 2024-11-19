#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "nvs_flash.h"
#include "mpu6050.h"
#include "time.h"
#include "esp_timer.h"

#include "lwip/sockets.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define DELAY(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static EventGroupHandle_t s_wifi_event_group;

// Global variable
const float RAD_TO_DEG = 57.2958;
const float DEG_TO_RAD = 0.0174533;

// Quarternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};

// UDP Variable
// #define WIFI_SSID "ruter"
// #define WIFI_PASS "caksa123"
#define WIFI_SSID "ini  wifii"
#define WIFI_PASS "12345678i"
#define PORT 3333
#define HOST_IP_ADDR  "192.168.137.1"//"192.168.0.110" // 192.168.43.32

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x1e

#include "HMC5883L.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533



// I2C functions implementation
 esp_err_t hmc5883l_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_NUM_0, HMC5883L_DEFAULT_ADDRESS, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t hmc5883l_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, HMC5883L_DEFAULT_ADDRESS, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}

bool hmc5883l_initialize(void) {
    // Configure sensor
    // The number of samples averaged per measured output is 8
    // Data Output Rate is 15Hz
    // Normal measurement configuration
    hmc5883l_write_byte(HMC5883L_RA_CONFIG_A, 0x70);
    
    // -1.3Ga-->+1.3Ga 1090 counts / Gauss
    hmc5883l_write_byte(HMC5883L_RA_CONFIG_B, 0x20);
    // hmc5883l_write_byte(HMC5883L_RA_CONFIG_B , HMC5883L_GAIN_820);
    // Single-Measurement Mode
    hmc5883l_write_byte(HMC5883L_RA_MODE, HMC5883L_MODE_CONTINUOUS);
    
    return true;
}

 bool hmc5883l_test_connection(void) {
    uint8_t data;
    esp_err_t ret = hmc5883l_read_bytes(HMC5883L_RA_ID_A, &data, 1);
    return (ret == ESP_OK && data == 0x48);
}

 bool hmc5883l_get_ready_status(void) {
    uint8_t status;
    hmc5883l_read_bytes(HMC5883L_RA_STATUS, &status, 1);
    return (status & 0x01) != 0;
}

void hmc5883l_get_heading(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t raw_data[6];
    hmc5883l_read_bytes(HMC5883L_RA_DATAX_H, raw_data, 6);
    printf("RAW:%u\n", (unsigned int)raw_data);
    *mx = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    *my = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    *mz = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

void hmc5883l_task(void *pvParameters) {
    // Initialize HMC5883L
    static const char *TAG = "HMC5883L";

    if (!hmc5883l_initialize()) {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        vTaskDelete(NULL);
    }

    // Verify the I2C connection
    if (!hmc5883l_test_connection()) {
        ESP_LOGE(TAG, "HMC5883L not found");
        vTaskDelete(NULL);
    }

    
    while(1) {
        // Read raw data from mag
        if (hmc5883l_get_ready_status()) {
            int16_t mx, my, mz;
            hmc5883l_get_heading(&mx, &my, &mz);
            // ESP_LOGI(TAG, "mag=%d %d %d", mx, my, mz);
            printf("x:%d y:%d z:%d\n", mx, my, mz);
            // mx = mx + CONFIG_MAGX;
            // my = my + CONFIG_MAGY;
            // mz = mz + CONFIG_MAGZ
        }

        vTaskDelay(10);
    }

    // Never reach here
    vTaskDelete(NULL);
}

static const char *TAG = "static_ip";

char payload[128] = "payload";

struct imu_data
{
    /* data */
    float x;
    float y;
    float z;
};

void i2c_init()
{
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

void mpu6050_init(mpu6050_handle_t mpu6050)
{
    if (mpu6050 == NULL)
    {
        printf("Failed to initialize MPU6050\n");
        return;
    }
    mpu6050_wake_up(mpu6050);
    mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_250DPS);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

void adc_init()
{
    esp_adc_cal_characteristics_t adc1_chars;

    // Config Pin and Voltage Range for Pin D33 or ADC1_CHANNEL_5
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    // Config ADC Width Range Value
    adc1_config_width(ADC_WIDTH_BIT_10);
    // Calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
}

void switch_init()
{
    /*Switch Mode Init pin : D5 or GPIO 5 */
    gpio_config_t io_conf1 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_5),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf1);

    /*Switch Mode Init pin : D4 or GPIO 4 */
    gpio_config_t io_conf2 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_4),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf2);

    /*Switch Mode Init pin : D19 or GPIO 19 */
    gpio_config_t io_conf3 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_19), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf3);

    /*Switch Mode Init pin : D26 or GPIO 26 */
    gpio_config_t io_conf4 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_26), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf4);

    /*Switch Mode Init pin : D15 or GPIO 15 */
    gpio_config_t io_conf5 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_15), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf5);
    
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
        esp_wifi_connect();
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);
    printf("wifi connecting");
    while(esp_wifi_connect() != ESP_OK){
    esp_wifi_start();
    esp_wifi_connect();
    printf(".");
    DELAY(100);
    }
}

static void udp_client_task(void *pvParameters)
{
    /* General Variable */
    // char *payload = (char *)pvParameters;
    static const char *TAG = "UDP SOCKET CLIENT";
    // char rx_buffer[128];
    // char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    while (1)
    {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1)
        {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                printf("%s \n", payload);

                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            // printf("%s ", payload);

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        // while(1){
        //     printf("%s \n", payload);
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        //     break;
        // }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax)
{
    // Ensure the input value is within the input range
    if (value < inputMin)
    {
        value = inputMin;
    }
    else if (value > inputMax)
    {
        value = inputMax;
    }

    // Map the input value to the output range
    float inputRange = inputMax - inputMin;
    float outputRange = outputMax - outputMin;
    return ((value - inputMin) * outputRange / inputRange) + outputMin;
}

int get_throttle()
{
    uint16_t pot_value, pot_pwm;
    uint16_t sum_1 = 0;

    // Multisampling
    for (uint8_t i = 0; i < 64; i++)
    {
        /* code */
        pot_value = adc1_get_raw(ADC1_CHANNEL_5);
        sum_1 += pot_value;
    }
    sum_1 /= 64;

    pot_pwm = mapValue(sum_1, 0, 200, 1000, 1900);
    return pot_pwm;
}

void Mahony_Update(float ax, float ay, float az, float gx, float gy, float gz, float delta_t)
{
    // PID Variable
    const float Kp = 30.0;
    const float Ki = 0.0;

    /* AHRS Variable */
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc;

    float ix = 0.0, iy = 0.0, iz = 0.0;
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    // tmp = 1;
    // ignore accelerometer if false (tested OK, SJR)
    if (tmp > 0.0)
    {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2];
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f)
        {
            ix += Ki * ex * delta_t; // integral error scaled by Ki
            iy += Ki * ey * delta_t;
            iz += Ki * ez * delta_t;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion, given by gyro term
    // rate of change = current orientation quaternion (qmult) gyro rate

    delta_t = 0.5 * delta_t;
    gx *= delta_t; // pre-multiply common factors
    gy *= delta_t;
    gz *= delta_t;
    qa = q[0];
    qb = q[1];
    qc = q[2];

    // add qmult*delta_t to current orientation
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
}

void sensor_read(struct imu_data *gyro, struct imu_data *acel, mpu6050_handle_t mpu6050)
{
    /* MPU6050 variable*/
    esp_err_t err_gyro, err_acce;
    mpu6050_raw_gyro_value_t gyro_data;
    mpu6050_raw_acce_value_t acce_data;

    /* Read Gyro Data */
    err_gyro = mpu6050_get_raw_gyro(mpu6050, &gyro_data);
    if (err_gyro != ESP_OK)
    {
        /* code */
        printf("Failed to get gyro data\n");
    }

    /* Read Acce Data */
    err_acce = mpu6050_get_raw_acce(mpu6050, &acce_data);
    if (err_acce != ESP_OK)
    {
        /* code */
        printf("Failed to get acce data\n");
    }

    /*Store Variable*/
    gyro->x = gyro_data.raw_gyro_x;
    gyro->y = gyro_data.raw_gyro_y;
    gyro->z = gyro_data.raw_gyro_z;

    acel->x = acce_data.raw_acce_x;
    acel->y = acce_data.raw_acce_y;
    acel->z = acce_data.raw_acce_z;
}

void start_process(void *pvParameters)
{
    /*Imu Data Raw*/
    struct imu_data gyro_raw;
    struct imu_data acel_raw;

    /*Imu data processed*/
    struct imu_data gyro_scaled;
    struct imu_data acel_scaled;
    struct imu_data gyro_offset;

    /*Roll Pitch Yaw variable*/
    float roll, pitch, yaw, yaw_1;
    uint16_t roll_pwm, pitch_pwm, yaw_pwm;

    /*Timing Variable*/
    float now = 0, last = 0, delta_t = 0;

    /*Constant Variable*/
    float const gyro_constant = 250.0 / 32768.0;
    float A_offset[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014};

    /* GPIO and ADC Variable*/
    uint16_t mode_1, mode_2, mode_pwm, arming, arming_pwm, throttle_pwm, magnet, magnet_pwm, poshold;

    // Initialize MPU6050
    mpu6050_handle_t mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    mpu6050_init(mpu6050);

    /* Payload Variable */
    // char *payload = (char *)pvParameters;

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++)
    {
        /* code */
        sensor_read(&gyro_raw, &acel_raw, mpu6050);
        gyro_offset.x += gyro_raw.x;
        gyro_offset.y += gyro_raw.y;
        gyro_offset.z += gyro_raw.z;
    }
    gyro_offset.x /= 2000;
    gyro_offset.y /= 2000;
    gyro_offset.z /= 2000;
    printf("Gyro Calibration done !!\n");

    while (true)
    {
        /* code */
        sensor_read(&gyro_raw, &acel_raw, mpu6050);

        /*Apply Offset & Scale Constant*/
        gyro_scaled.x = (gyro_raw.x - gyro_offset.x) * DEG_TO_RAD * gyro_constant;
        gyro_scaled.y = (gyro_raw.y - gyro_offset.y) * DEG_TO_RAD * gyro_constant;
        gyro_scaled.z = (gyro_raw.z - gyro_offset.z) * DEG_TO_RAD * gyro_constant;

        acel_scaled.x = (acel_raw.x - A_offset[0]) * A_offset[3];
        acel_scaled.y = (acel_raw.y - A_offset[1]) * A_offset[4];
        acel_scaled.z = (acel_raw.z - A_offset[2]) * A_offset[5];

        /*Get ESP Timing*/
        now = esp_timer_get_time();
        delta_t = (now - last) * 0.000001;
        last = now;
        // printf("%f, ", delta_t);
        /*Call Mahony Update*/
        Mahony_Update(acel_scaled.x, acel_scaled.y, acel_scaled.z, gyro_scaled.x, gyro_scaled.y, gyro_scaled.z, delta_t);

        /*Get Roll Pitch Yaw Angle*/
        roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
        pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
        // conventional yaw increases clockwise from North. Note that the MPU-6050 does not know where True North is.
        yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

        // Convert to degrees
        yaw *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;

        /*Limit Roll  Angle*/
        if (roll < -70)
        {
            roll = -70;
        }
        else if (roll > 70)
        {
            roll = 70;
        }

        /*Limit Pitch Angle*/
        if (pitch < -65)
        {
            pitch = -65;
        }
        else if (pitch > 65)
        {
            pitch = 65;
        }

        /*Mapping degree to PWM*/
        roll_pwm = mapValue(roll, -45, 45, 1000, 2000);
        pitch_pwm = mapValue(pitch, -45, 45, 2000, 1000);
        throttle_pwm = get_throttle();
       // yaw_pwm = 1500;

        /* 0 to 360 degrees with Relative North Position */
        // if (yaw < 0)
        //     yaw += 360.0; // compass circle
        // // correct for local magnetic declination here

        yaw_1 = yaw;
        // Ensure yaw angle is within the range -180 to 180 degrees
        if (yaw > 180.0)
        {
            yaw -= 360.0;
        }
        else if (yaw < -180.0)
        {
            yaw += 360.0;
        }

        // Ensure yaw angle is within the range -90 to 90 degrees
        if (yaw > 90.0)
        {
            yaw -= 180.0;
        }
        else if (yaw < -90.0)
        {
            yaw += 180.0;
        }

        // Limit PWM Value
        if (yaw_1 < -90)
        {
            yaw_pwm = 1000;
        }
        else if (yaw_1 > 90)
        {
            yaw_pwm = 2000;
        }
        else
        {
            yaw_pwm = ((yaw + 90) / 180) * (2000 - 1000) + 1000;
        }

        // Deadzone
        if (yaw_pwm > 1450 && yaw_pwm < 1550){
            yaw_pwm = 1500;
        }else if (yaw_pwm > 1450){
            yaw_pwm += 50;
        }else if (yaw_pwm < 1550){
            yaw_pwm -= 50;
        }

        /*Switch Mode*/
        mode_1 = gpio_get_level(GPIO_NUM_5);
        mode_2 = gpio_get_level(GPIO_NUM_19);
        if (mode_1 == 1 && mode_2 == 0)
        {
            mode_pwm = 2000; // pos hold
            // if (throttle_pwm > 1430 && throttle_pwm < 1570){
            //     throttle_pwm = 1500;
            // }
        }
        else if (mode_1 == 0 && mode_2 == 1)
        {
            mode_pwm = 1000; // Stabilize
        }
        else
        {
            mode_pwm = 1500; // Land
        }

        /*Switch Arming*/
        arming = gpio_get_level(GPIO_NUM_4);
        if (arming == 1)
        {
            arming_pwm = 2000;
        }
        else
        {
            arming_pwm = 1000;
        }

        /*Switch Magnet*/
        magnet = gpio_get_level(GPIO_NUM_26);
        if (magnet == 1)
        {
            magnet_pwm = 2000;
        }
        else
        {
            magnet_pwm= 1000;
        }

        /*Switch Force Poshold*/
        poshold = gpio_get_level(GPIO_NUM_15);
        if (poshold == 1)
        {
            yaw_pwm = 1500;
        }
        
        sprintf(payload, "r%dp%dy%dm%d\n", roll_pwm, pitch_pwm, yaw_pwm, mode_pwm);
        // printf("%d\n", yaw_pwm);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}



void app_main(void)
{
    i2c_init();
    hmc5883l_initialize();
    // adc_init();
    // switch_init();
    // wifi_connection();

    vTaskDelay(500 / portTICK_PERIOD_MS);
    // xTaskCreate(start_process, "start_process", 4096, NULL, 5, NULL);
    xTaskCreate(hmc5883l_task, "hmc5883l_task", 1024*8, NULL, 5, NULL);
    // xTaskCreate(udp_client_task, "udp_cilent_task", 4096, NULL, 4, NULL);
}