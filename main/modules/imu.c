#include <stdio.h>
#include <math.h>

#include "esp_err.h"
#include "esp_timer.h"

#include "modules/imu.h"
#include "modules/filters.h"
// Global constant variables
static const float RAD_TO_DEG = 57.2958;
static const float DEG_TO_RAD = 0.0174533;
static const float gyro_constant = 250.0 / 32768.0;

static struct imu_data A_offset1 = { 265.0, -80.0, -700.0 };
static struct imu_data A_offset2 = { 0.994, 1.000, 1.014 };

// IMU operator
void imu_add(struct imu_data *a, struct imu_data b) {
    a->x += b.x;
    a->y += b.y;
    a->z += b.z;
}

void imu_substract(struct imu_data *a, struct imu_data b) {
    a->x -= b.x;
    a->y -= b.y;
    a->z -= b.z;
}

struct imu_data imu_substract_return(struct imu_data a, struct imu_data b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;

    return a;
}

void imu_multiply_single(struct imu_data *a, float b) {
    a->x *= b;
    a->y *= b;
    a->z *= b;
}

struct imu_data imu_multiply_single_return(struct imu_data a, float b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;

    return a;
}

void imu_multiply(struct imu_data *a, struct imu_data b) {
    a->x *= b.x;
    a->y *= b.y;
    a->z *= b.z;
}

struct imu_data imu_multiply_return(struct imu_data a, struct imu_data b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;

    return a;
}

void imu_divide_single(struct imu_data *a, float b) {
    a->x /= b;
    a->y /= b;
    a->z /= b;
}

// IMU processing
struct full_imu_data create_full_imu_data() {
    struct full_imu_data data = { 0 };
    data.q[0] = 1.0;
    return data;
}

esp_err_t imu_read_raw(mpu6050_handle_t mpu6050, struct imu_data *gyro, struct imu_data *acce) {
    /* MPU6050 variable*/
    esp_err_t err;
    mpu6050_raw_gyro_value_t gyro_data;
    mpu6050_raw_acce_value_t acce_data;

    /* Read Gyro Data */
    err = mpu6050_get_raw_gyro(mpu6050, &gyro_data);
    if (err != ESP_OK) {
        /* code */
        printf("Failed to get gyro data\n");
        return err;
    }

    /* Read Acce Data */
    err = mpu6050_get_raw_acce(mpu6050, &acce_data);
    if (err != ESP_OK) {
        /* code */
        printf("Failed to get acce data\n");
        return err;
    }

    /*Store Variable*/
    gyro->x = gyro_data.raw_gyro_x;
    gyro->y = gyro_data.raw_gyro_y;
    gyro->z = gyro_data.raw_gyro_z;

    acce->x = acce_data.raw_acce_x;
    acce->y = acce_data.raw_acce_y;
    acce->z = acce_data.raw_acce_z;

    return err;
}

mpu6050_handle_t imu_init(i2c_port_t port, const uint16_t dev_addr) {
    printf("WAKING UP %d\n", dev_addr);
    mpu6050_handle_t mpu6050 = mpu6050_create(port, dev_addr);

    if (mpu6050 == NULL) {
        printf("Failed to initialize\n");
    }

    esp_err_t err = mpu6050_wake_up(mpu6050);
    if (err != ESP_OK) {
        printf("Failed to wake IMU, %d\n", dev_addr);
    }

    err = mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_250DPS);
    if (err != ESP_OK) {
        printf("Failed to config IMU, %d\n", dev_addr);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return mpu6050;
}

void imu_read(mpu6050_handle_t mpu6050, struct full_imu_data *data) {
    esp_err_t err = imu_read_raw(mpu6050, &data->gyro, &data->acce);
    if (err != ESP_OK) {
        printf("Failed to read IMU data\n");
        return;
    }

    // Apply offsets and scaling
    imu_substract(&data->gyro, data->offset);
    imu_multiply_single(&data->gyro, DEG_TO_RAD * gyro_constant);
    imu_substract(&data->acce, A_offset1);
    imu_multiply(&data->acce, A_offset2);

    // Update timing
    data->now = esp_timer_get_time();
    data->delta_t = (data->now - data->last) * 0.000001;
    data->last = data->now;
}

void imu_process(struct full_imu_data *data) {
    quaternion_to_euler(data->q, &data->processed);

    // Convert to degrees
    imu_multiply_single(&data->processed, RAD_TO_DEG);
}
