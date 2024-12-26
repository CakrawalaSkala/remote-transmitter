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