#include "mpu6050.h"

struct imu_data {
    float x;
    float y;
    float z;
};

struct full_imu_data {
    struct imu_data offset;
    float now, last, delta_t;

    struct imu_data acce;
    struct imu_data gyro;

    // Quaternion
    float q[4];
    struct imu_data processed;
};

// IMU operator
void imu_add(struct imu_data *a, struct imu_data b);
void imu_substract(struct imu_data *a, struct imu_data b);
void imu_multiply_single(struct imu_data *a, float b);
struct imu_data imu_multiply_single_return(struct imu_data a, float b);
void imu_multiply(struct imu_data *a, struct imu_data b);
struct imu_data imu_multiply_return(struct imu_data a, struct imu_data b);
void imu_divide_single(struct imu_data *a, float b);

mpu6050_handle_t imu_init(i2c_port_t port, const uint16_t dev_addr);
esp_err_t imu_read_raw(mpu6050_handle_t mpu6050, struct imu_data *gyro, struct imu_data *acce);
void imu_read(mpu6050_handle_t mpu6050, struct full_imu_data *data);