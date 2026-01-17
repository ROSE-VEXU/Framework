#include "vex.h"

namespace BlackMagic {

PID::PID(float kP, IntegralConfig cI, float kD): total_error(0), prev_error(0), prev_output(0) {
    this->kP = kP;
    this->cI = cI;
    this->kD = kD;
    this->config = { PID_SETTING_DISABLE, PID_SETTING_DISABLE, PID_SETTING_DISABLE, PID_SETTING_DISABLE };
}

PID::PID(float kP, IntegralConfig cI, float kD, PIDConfig pid_config): total_error(0), prev_error(0), prev_output(0) {
    this->kP = kP;
    this->cI = cI;
    this->kD = kD;
    this->config = pid_config;
}

float PID::slew(float prev_value, float value) {
    float value_delta = fabs(value) - fabs(prev_value);

    if (fabs(prevValue) < fabs(value)) { // Accelerating
        if (value_delta >= config.accel_slew_step && config.accel_slew_step != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prev_value + Utils::sign(value)*config.accel_slew_step;
        }
    } else { // Decelerating
        if (fabs(value_delta) >= config.decel_slew_step && config.decel_slew_step != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prev_value - Utils::sign(value)*config.decel_slew_step;
        }
    }

    return value;
}

float PID::getNextValue(float err) {
    float result = kP*err + kD*(err-prev_error);

    if (fabs(err) < cI.start_integral_threshold) {
        total_error += err;
        total_error = Utils::clamp(total_error, -cI.max_integral, cI.max_integral);
        result += cI.kI*total_error;
    }

    prev_error = err;

    result = slew(prev_output, result);
    result = Utils::sign(result) * Utils::clamp(
        fabs(result),
        config.min_speed == PID_SETTING_DISABLE ? 0.0 : config.min_speed,
        config.max_speed == PID_SETTING_DISABLE ? 100.0 : config.max_speed
    );

    prev_output = result;
    return result;
}

void PID::reset() {
    prev_error = 0;
    total_error = 0;
    prev_output = 0;
}
    
};
