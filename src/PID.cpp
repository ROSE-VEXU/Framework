#include "vex.h"

namespace BlackMagic {

PID PID::ZERO_PID = { 0.0, { 0.0, 0.0, 0.0 }, 0.0, { 0, 0 }, IErrorProvider<float>() };

template<typename ErrorType>
PID<ErrorType>::PID(float kP, IntegralConfig cI, float kD, SettleConfig settle_config, IErrorProvider<ErrorType> error_provider):
    kP(kP), cI(cI), kD(kD),
    accel_slew(PID_SETTING_DISABLE), settle_config(settle_config), error_provider(error_provider),
    max_speed(0.0),
    total_error(0), prev_error(0), prev_output(0) {
}

template<typename ErrorType>
PID<ErrorType>::PID(float kP, IntegralConfig cI, float kD, float accel_slew, SettleConfig settle_config, IErrorProvider<ErrorType> error_provider): 
    kP(kP), cI(cI), kD(kD),
    accel_slew(accel_slew), settle_config(settle_config), error_provider(error_provider),
    max_speed(0.0),
    total_error(0), prev_error(0), prev_output(0) {
}

template<typename ErrorType>
float PID<ErrorType>::slew(float prev_value, float value) {
    float value_delta = fabs(value) - fabs(prev_value);

    if (fabs(prev_value) < fabs(value)) { // Accelerating
        if (value_delta >= accel_slew && accel_slew != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prev_value + (Utils::sign(value) * accel_slew);
        }
    } 

    return value;
}

template<typename ErrorType>
float PID<ErrorType>::getNextValue(ErrorType target) {
    float err = error_provider.getError(target);
    float result = kP*err + kD*(err-prev_error);

    if (fabs(err) < cI.start_integral_threshold) {
        total_error += err;
        total_error = Utils::clamp(total_error, -cI.max_integral, cI.max_integral);
        result += cI.kI*total_error;
    }

    prev_error = err;

    result = slew(prev_output, result);
    result = Utils::clamp(result, -max_speed, max_speed);

    prev_output = result;
    return result;
}

template<typename ErrorType>
void PID<ErrorType>::setMaxSpeed(float max_speed) {
    this->max_speed = max_speed;
}

template<typename ErrorType>
float PID<ErrorType>::getMaxSpeed() {
    return max_speed;
}

template<typename ErrorType>
void PID<ErrorType>::reset() {
    prev_error = 0.0;
    total_error = 0.0;
    prev_output = 0.0;
    max_speed = 0.0;
}
    
};
