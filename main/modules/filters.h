#include "modules/imu.h"
#include "dsps_math.h"

// Filter configurations (constants)
struct mahony_config {
    float Kp;
    float Ki;
};

struct kalman_config {
    float Q_angle;    // Process noise for angle
    float Q_bias;     // Process noise for bias
    float R_measure;  // Measurement noise
};

// Filter states (values that need to persist between updates)
struct mahony_state {
    struct imu_data integral;  // Only the integral needs to persist
};

struct kalman_state {
    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2];      // Error covariance matrix
};

// Filter containers
struct mahony_filter {
    struct mahony_state state;
    const struct mahony_config *config;
};

struct kalman_filter {
    struct kalman_state states[3];  // One for each axis
    const struct kalman_config *config;
};

// Factory functions
struct mahony_filter *create_mahony_filter(const struct mahony_config *config);
struct kalman_filter *create_kalman_filter(const struct kalman_config *config);
// Quaternion operations
void quaternion_normalize(float q[4]);
void quaternion_update(float q[4], struct imu_data gyro, float delta_t);
void quaternion_to_euler(float q[4], struct imu_data *euler);

// Direct filter application functions
void apply_mahony_filter(struct mahony_filter *filter, struct imu_data *gyro,
    struct imu_data *acce, float delta_t, float q[4]);
void apply_kalman_filter(struct kalman_filter *filter, struct imu_data *gyro,
    struct imu_data *acce, float delta_t, float q[4]);