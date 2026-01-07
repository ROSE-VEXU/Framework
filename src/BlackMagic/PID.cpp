#include "vex.h"

namespace BlackMagic {

PID::PID(float kP, float kI, float kD): prevError(0), totalError(0), prevOutput(0) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->accelSlewStep = STEP_DISABLE;
    this->decelSlewStep = STEP_DISABLE;
}

PID::PID(float kP, float kI, float kD, float accelSlewStep, float decelSlewStep): prevError(0), totalError(0), prevOutput(0) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->accelSlewStep = accelSlewStep;
    this->decelSlewStep = decelSlewStep;
}

float PID::slew(float prevValue, float value) {
    float valueDelta = fabs(value) - fabs(prevValue);
    if (fabs(prevValue) < fabs(value)) { // Accelerating
        if (valueDelta >= accelSlewStep) { // Limit output delta to config'ed step if accelerating too fast
            value = prevValue + direction(value)*accelSlewStep;
        }
    } else { // Decelerating
        if (fabs(valueDelta) >= decelSlewStep) { // Limit output delta to config'ed step if accelerating too fast
            value = prevValue - direction(value)*decelSlewStep;
        }
    }

    return value;
}

float PID::getNextValue(float err) {
    totalError += err;

    float result = kP*err + kI*totalError + kD*(err-prevError);
    prevError = err;

    if (accelSlewStep != STEP_DISABLE || decelSlewStep != STEP_DISABLE) {
        result = slew(prevOutput, result);
    }

    prevOutput = result;
    return result;
}

void PID::reset() {
    prevError = 0;
    totalError = 0;
    prevOutput = 0;
}

const int PID::direction(float value) {
    return (value < 0) ? -1 : 1;
}
    
};
