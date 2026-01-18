#ifndef PID_H
#define PID_H

#define PID_SETTING_DISABLE -2048

namespace BlackMagic {

struct IntegralConfig {
    float kI;
    float max_integral;
    float start_integral_threshold;
};

struct PIDConfig {
    float min_speed;
    float max_speed;
    float accel_slew_step;
    float decel_slew_step;
};

class PID {
public:
    static PID ZERO_PID;

    PID(float kP, IntegralConfig cI, float kD);
    PID(float kP, IntegralConfig cI, float kD, PIDConfig pid_config);
    float getNextValue(float err);
    void reset();
private:
    float kP;
    IntegralConfig cI;
    float kD;
    PIDConfig config;
    float total_error;
    float prev_error;
    float prev_output;

    const int direction(float value);
    float slew(float prev_value, float value);
};

};

#endif