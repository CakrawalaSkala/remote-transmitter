#include <stdio.h>
#include <math.h>

#include "esp_err.h"
#include "esp_timer.h"

#include "imu.h"

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
void Mahony_Update(float q[4], struct imu_data gyro, struct imu_data acce, float delta_t) {
    // PID Variable
    const float Kp = 30.0;
    const float Ki = 0.0;

    /* AHRS Variable */
    float recipNorm;
    struct imu_data v;
    struct imu_data e;
    float qa, qb, qc;

    struct imu_data i;
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = acce.x * acce.x + acce.y * acce.y + acce.z * acce.z;
    // tmp = 1;
    // ignore accelerometer if false (tested OK, SJR)
    if (tmp > 0.0) {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        imu_multiply_single(&acce, recipNorm);

        // Estimated direction of gravity in the body frame (factor of two divided out)
        v.x = q[1] * q[3] - q[0] * q[2];
        v.y = q[0] * q[1] + q[2] * q[3];
        v.z = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        e.x = (acce.y * v.z - acce.z * v.y);
        e.y = (acce.z * v.x - acce.x * v.z);
        e.z = (acce.x * v.y - acce.y * v.x);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
            // integral error scaled by Ki
            imu_add(&i, imu_multiply_single_return(e, Ki * delta_t));

            // apply integral feedback
            imu_add(&gyro, i);
        }

        // Apply proportional feedback to gyro term
        imu_add(&gyro, imu_multiply_single_return(e, Kp));
    }

    // Integrate rate of change of quaternion, given by gyro term
    // rate of change = current orientation quaternion (qmult) gyro rate

    delta_t *= 0.5;
    imu_multiply_single(&gyro, delta_t);
    qa = q[0];
    qb = q[1];
    qc = q[2];

    // add qmult*delta_t to current orientation
    q[0] += (-qb * gyro.x - qc * gyro.y - q[3] * gyro.z);
    q[1] += (qa * gyro.x + qc * gyro.z - q[3] * gyro.y);
    q[2] += (qa * gyro.y - qb * gyro.z + q[3] * gyro.x);
    q[3] += (qa * gyro.z + qb * gyro.y - qc * gyro.x);

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
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
    mpu6050_handle_t mpu6050 = mpu6050_create(port, dev_addr);

    mpu6050_wake_up(mpu6050);
    mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_250DPS);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    return mpu6050;
}

void imu_read(mpu6050_handle_t mpu6050, struct full_imu_data *data) {
    esp_err_t err = imu_read_raw(mpu6050, &data->gyro, &data->acce);
    if (err != ESP_OK) {
        printf("Failed to read IMU data\n");
        return;
    }

    /*Apply Offset & Scale Constant*/
    // Gyro
    imu_substract(&data->gyro, data->offset);
    imu_multiply_single(&data->gyro, DEG_TO_RAD * gyro_constant);

    // Acce
    imu_substract(&data->acce, A_offset1);
    imu_multiply(&data->acce, A_offset2);

    data->now = esp_timer_get_time();
    data->delta_t = (data->now - data->last) * 0.000001;
    data->last = data->now;

    Mahony_Update(data->q, data->gyro, data->acce, data->delta_t);

    data->processed.x = asin(2.0 * (data->q[0] * data->q[2] - data->q[1] * data->q[3]));
    data->processed.y = atan2((data->q[0] * data->q[1] + data->q[2] * data->q[3]), 0.5 - (data->q[1] * data->q[1] + data->q[2] * data->q[2]));
    // conventional yaw increases clockwise from North. Note that the MPU-6050 does not know where True North is.
    data->processed.z = -atan2((data->q[1] * data->q[2] + data->q[0] * data->q[3]), 0.5 - (data->q[2] * data->q[2] + data->q[3] * data->q[3]));

    // Convert to degrees
    imu_multiply_single(&data->processed, RAD_TO_DEG);
}