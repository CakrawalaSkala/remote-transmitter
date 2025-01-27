#include "iot_button.h"

button_handle_t init_btn(gpio_num_t gpio, button_cb_t cb, void *usr_data) {
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = gpio,
            .active_level = 0,
        },
    };

    button_handle_t btn = iot_button_create(&gpio_btn_cfg);
    iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, cb, usr_data);

    return btn;
}

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    if (value < inputMin) return outputMin;
    else if (value > inputMax) return outputMax;

    // Map the input value to the output range
    float inputRange = inputMax - inputMin;
    float outputRange = outputMax - outputMin;
    return ((value - inputMin) * outputRange / inputRange) + outputMin;
}

void applyDeadzone(float *value, float center, float deadzone) {
    float upper = center + deadzone;
    float lower = center - deadzone;

    if (*value < lower) *value += deadzone;
    else if (*value > upper) *value -= deadzone;
    else *value = center;
}