#ifndef PID_H
#define PID_H

#define STEP_DISABLE 127

namespace BlackMagic {

class PID {
public:
    PID(float kP, float kI, float kD);
    PID(float kP, float kI, float kD, float accelSlewStep, float decelSlewStep);
    float slew(float prevValue, float value);
    float getNextValue(float err);
    void reset();
private:
    float kP;
    float kI;
    float kD;
    float accelSlewStep;
    float decelSlewStep;
    float totalError;
    float prevError;
    float prevOutput;

    const int direction(float value);
};

};

#endif