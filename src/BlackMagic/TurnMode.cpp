#include "vex.h"

namespace BlackMagic {

void TurnMode::setTarget(float targetHeading) {
    this->targetHeading = targetHeading;
}

void TurnMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {

}

bool TurnMode::hasSettled() {
    return false;
}

DriveSpeeds TurnMode::getSpeeds() {
    return { 0.0, 0.0 };
}

};
