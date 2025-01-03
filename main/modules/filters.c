#include <math.h>
#include "filters.h"

// Quaternion operations
void quaternion_normalize(float q[4]) {
    float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

void quaternion_update(float q[4], struct imu_data gyro, float delta_t) {
    float qa = q[0];
    float qb = q[1];
    float qc = q[2];

    delta_t *= 0.5;
    gyro.x *= delta_t;
    gyro.y *= delta_t;
    gyro.z *= delta_t;

    q[0] += (-qb * gyro.x - qc * gyro.y - q[3] * gyro.z);
    q[1] += (qa * gyro.x + qc * gyro.z - q[3] * gyro.y);
    q[2] += (qa * gyro.y - qb * gyro.z + q[3] * gyro.x);
    q[3] += (qa * gyro.z + qb * gyro.y - qc * gyro.x);

    quaternion_normalize(q);
}

void quaternion_to_euler(float q[4], struct imu_data *euler) {
    euler->x = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    euler->y = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    euler->z = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
}

// Default configurations
static const struct mahony_config default_mahony_config = {
    .Kp = 30.0f,
    .Ki = 0.0f
};

static const struct kalman_config default_kalman_config = {
.Q_angle = 0.01f,
.Q_bias = 0.005f,
.R_measure = 0.02f
};

struct mahony_filter create_mahony_filter(const struct mahony_config *config) {
    struct mahony_filter filter;
    filter.config = config ? config : &default_mahony_config;
    filter.state.integral.x = 0.0f;
    filter.state.integral.y = 0.0f;
    filter.state.integral.z = 0.0f;
    return filter;
}

static void kalman_init(struct kalman_state *state, const struct kalman_config *config) {
    // Initialize state vector
    state->angle = 0.0f;
    state->bias = 0.0f;

    // Initialize error covariance matrix
    state->P[0][0] = 0.0f;
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 0.0f;
}

struct kalman_filter create_kalman_filter(const struct kalman_config *config) {
    struct kalman_filter filter;

    filter.config = config ? config : &default_kalman_config;
    for (int i = 0; i < 3; i++) {
        kalman_init(&filter.states[i], filter.config);
    }

    return filter;
}

// Mahony implementation
void apply_mahony_filter(struct mahony_filter *filter, struct imu_data *gyro,
    struct imu_data *acce, float delta_t, float q[4]) {
    float recipNorm;
    struct imu_data v;
    struct imu_data e;
    float tmp;

    // Check valid accelerometer data
    tmp = acce->x * acce->x + acce->y * acce->y + acce->z * acce->z;
    if (tmp > 0.0) {
        // Normalize accelerometer
        recipNorm = 1.0 / sqrt(tmp);
        acce->x *= recipNorm;
        acce->y *= recipNorm;
        acce->z *= recipNorm;

        // Estimated direction of gravity
        v.x = q[1] * q[3] - q[0] * q[2];
        v.y = q[0] * q[1] + q[2] * q[3];
        v.z = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error calculation
        e.x = (acce->y * v.z - acce->z * v.y);
        e.y = (acce->z * v.x - acce->x * v.z);
        e.z = (acce->x * v.y - acce->y * v.x);

        // Apply feedback
        if (filter->config->Ki > 0.0f) {
            filter->state.integral.x += e.x * filter->config->Ki * delta_t;
            filter->state.integral.y += e.y * filter->config->Ki * delta_t;
            filter->state.integral.z += e.z * filter->config->Ki * delta_t;

            gyro->x += filter->state.integral.x;
            gyro->y += filter->state.integral.y;
            gyro->z += filter->state.integral.z;
        }

        gyro->x += e.x * filter->config->Kp;
        gyro->y += e.y * filter->config->Kp;
        gyro->z += e.z * filter->config->Kp;
    }
    quaternion_update(q, *gyro, delta_t);
}

void apply_kalman_filter(struct kalman_filter *filter, struct imu_data *gyro,
    struct imu_data *acce, float delta_t, float q[4]) {
// Process each axis
    float angles[3];
    float rates[3] = { gyro->x, gyro->y, gyro->z };

    // Calculate angles from accelerometer
    angles[0] = atan2f(acce->y, acce->z);  // Roll
    angles[1] = atan2f(-acce->x, sqrtf(acce->y * acce->y + acce->z * acce->z));  // Pitch
    angles[2] = 0;  // Yaw can't be determined from accelerometer alone

    // Process each axis with Kalman filter
    for (int i = 0; i < 3; i++) {
        struct kalman_state *state = &filter->states[i];

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        state->rate = rates[i] - state->bias;
        state->bias = delta_t * state->rate;

        // Update error covariance matrix P = F*P*F' + Q
        /* Step 2 */
        state->P[0][0] += delta_t * (delta_t * state->P[1][1] - state->P[0][1] - state->P[1][0] + filter->config->Q_angle);
        state->P[0][1] -= delta_t * state->P[1][1];
        state->P[1][0] -= delta_t * state->P[1][1];
        state->P[1][1] += filter->config->Q_bias * delta_t;

        // Update step
        // Innovation covariance
        /* Step 4 */
        float S = state->P[0][0] + filter->config->R_measure;
        /* Step 5 */
        float K[2]; // Kalman gain - This is a 2x1 vector
        K[0] = state->P[0][0] / S;
        K[1] = state->P[1][0] / S;

        // Innovation
        /* Step 3 */
        float y = angles[i] - state->angle;

        // Update state
        /* Step 6 */
        state->angle += K[0] * y;
        state->bias += K[1] * y;

        // Update error covariance matrix
        /* Step 7 */
        float P00_temp = state->P[0][0];
        float P01_temp = state->P[0][1];

        state->P[0][0] -= K[0] * P00_temp;
        state->P[0][1] -= K[0] * P01_temp;
        state->P[1][0] -= K[1] * P00_temp;
        state->P[1][1] -= K[1] * P01_temp;
    }

    // Convert filtered angles to quaternion
    float cr = cosf(filter->states[0].angle * 0.5f);
    float sr = sinf(filter->states[0].angle * 0.5f);
    float cp = cosf(filter->states[1].angle * 0.5f);
    float sp = sinf(filter->states[1].angle * 0.5f);
    float cy = cosf(filter->states[2].angle * 0.5f);
    float sy = sinf(filter->states[2].angle * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    quaternion_normalize(q);
}