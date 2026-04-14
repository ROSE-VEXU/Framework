#ifndef PID_H
#define PID_H

#include "Drivetrain.h"
#include "ErrorProvider.h"
#include <regex>
#include <type_traits>
#define PID_SETTING_DISABLE -2048

namespace BlackMagic {

struct IntegralConfig {
    float kI;
    float max_integral;
    float start_integral_threshold;

    IntegralConfig(float kI): kI(kI), max_integral(0.0), start_integral_threshold(0.0) {};
    IntegralConfig(float kI, float max_integral, float start_integral_threshold): kI(kI), max_integral(max_integral), start_integral_threshold(start_integral_threshold) {};
};

class PID {
public:
    static PID ZERO_PID;

    PID(float kP, IntegralConfig cI, float kD, IErrorProvider& error_provider);
    PID(float kP, IntegralConfig cI, float kD, float accel_slew, IErrorProvider& error_provider);
    float getNextValue(float err);
    void setMaxSpeed(float max_speed);
    float getMaxSpeed();
    bool hasSettled();
    void reset();
private:
    float kP;
    IntegralConfig cI;
    float kD;
    float accel_slew;
    IErrorProvider* error_provider;
    float max_speed;
    float total_error;
    float prev_error;
    float prev_output;

    const int direction(float value);
    float slew(float prev_value, float value);
};

};

#endif