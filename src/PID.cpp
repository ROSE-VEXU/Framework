#include "vex.h"

namespace BlackMagic {

PID PID::ZERO_PID = { 0.0, { 0.0, 0.0, 0.0 }, 0.0 };

PID::PID(float kP, IntegralConfig cI, float kD):
    kP(kP), cI(cI), kD(kD),
    accel_slew(PID_SETTING_DISABLE), max_speed(0.0),
    total_error(0), prev_error(0), prev_output(0) {
}

PID::PID(float kP, IntegralConfig cI, float kD, float accel_slew): 
    kP(kP), cI(cI), kD(kD),
    accel_slew(accel_slew), max_speed(0.0),
    total_error(0), prev_error(0), prev_output(0) {
}

float PID::slew(float prev_value, float value) {
    float value_delta = fabs(value) - fabs(prev_value);

    if (fabs(prev_value) < fabs(value)) { // Accelerating
        if (value_delta >= accel_slew && accel_slew != PID_SETTING_DISABLE) { // Limit output delta to config'ed step if accelerating too fast
            value = prev_value + Utils::sign(value)*config.accel_slew_step;
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

    prev_output = result;
    return result;
}

void PID::setMaxSpeed(float max_speed) {
    this->max_speed = max_speed;
}

void PID::reset() {
    prev_error = 0.0;
    total_error = 0.0;
    prev_output = 0.0;
    max_speed = 0.0;
}
    
};
