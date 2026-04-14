#ifndef PID_H
#define PID_H

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

struct SettleConfig {
    int max_settle_count;
    int reset_settle_threshold;
};

template <typename ErrorType>
class PID {
    static_assert(std::is_convertible<ErrorType, float>::value, "ErrorType must be convertible to float for PID error calculations.");

public:
    static PID ZERO_PID;

    PID(float kP, IntegralConfig cI, float kD, SettleConfig settle_config, IErrorProvider<ErrorType> error_provider);
    PID(float kP, IntegralConfig cI, float kD, float accel_slew, SettleConfig settle_config, IErrorProvider<ErrorType> error_provider);
    float getNextValue(ErrorType target);
    void setMaxSpeed(float max_speed);
    float getMaxSpeed();
    void reset();
private:
    float kP;
    IntegralConfig cI;
    float kD;
    float accel_slew;
    SettleConfig settle_config;
    IErrorProvider<ErrorType> error_provider;
    float max_speed;
    float total_error;
    float prev_error;
    float prev_output;

    const int direction(float value);
    float slew(float prev_value, float value);
};

};

#endif