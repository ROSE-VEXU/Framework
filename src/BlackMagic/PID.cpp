#include "vex.h"

namespace BlackMagic {

PID::PID(float kP, float kI, float kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->accelSlewStep = STEP_DISABLE;
    this->decelSlewStep = STEP_DISABLE;
}

PID::PID(float kP, float kI, float kD, float accelSlewStep, float decelSlewStep) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->accelSlewStep = accelSlewStep;
    this->decelSlewStep = decelSlewStep;
}

float PID::slew(float prevValue, float value) {
    return 0.0; // TODO - Implement PID logic
}

float PID::getNextValue(float inputValue) {
    return 0.0; // TODO - Implement PID logic
}

void PID::reset() {

}
    
};
