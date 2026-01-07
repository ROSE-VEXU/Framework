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
    PID(float kP, IntegralConfig cI, float kD);
    PID(float kP, IntegralConfig cI, float kD, PIDConfig pid_config);
    float getNextValue(float err);
    void reset();
private:
    float kP;
    IntegralConfig cI;
    float kD;
    PIDConfig config;
    float totalError;
    float prevError;
    float prevOutput;

    const int direction(float value);
    float slew(float prevValue, float value);
};

};

#endif