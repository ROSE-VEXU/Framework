#include "vex.h"
#include <algorithm>

namespace BlackMagic {

PID::PID(float kP, IntegralConfig cI, float kD): totalError(0), prevError(0), prevOutput(0) {
    this->kP = kP;
    this->cI = cI;
    this->kD = kD;
    this->config = { PID_SETTING_DISABLE, PID_SETTING_DISABLE, PID_SETTING_DISABLE, PID_SETTING_DISABLE };
}

PID::PID(float kP, IntegralConfig cI, float kD, PIDConfig pid_config): totalError(0), prevError(0), prevOutput(0) {
    this->kP = kP;
    this->cI = cI;
    this->kD = kD;
    this->config = pid_config;
}

float PID::slew(float prevValue, float value) {
    float valueDelta = fabs(value) - fabs(prevValue);

    if (fabs(prevValue) < fabs(value)) { // Accelerating
        if (valueDelta >= config.accel_slew_step && config.accel_slew_step != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prevValue + Utils::sign(value)*config.accel_slew_step;
        }
    } else { // Decelerating
        if (fabs(valueDelta) >= config.decel_slew_step && config.decel_slew_step != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prevValue - Utils::sign(value)*config.decel_slew_step;
        }
    }

    return value;
}

float PID::getNextValue(float err) {
    float result = kP*err + kD*(err-prevError);

    if (fabs(err) < cI.start_integral_threshold) {
        totalError += err;
        totalError = Utils::clamp(totalError, -cI.max_integral, cI.max_integral);
        result += cI.kI*totalError;
    }

    prevError = err;

    result = slew(prevOutput, result);
    result = Utils::sign(result) * Utils::clamp(
        fabs(result),
        config.min_speed == PID_SETTING_DISABLE ? 0.0 : config.min_speed,
        config.min_speed == PID_SETTING_DISABLE ? 100.0 : config.max_speed
    );

    prevOutput = result;
    return result;
}

void PID::reset() {
    prevError = 0;
    totalError = 0;
    prevOutput = 0;
}
    
};
