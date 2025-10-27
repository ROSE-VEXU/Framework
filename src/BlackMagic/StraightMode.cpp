#include "vex.h"

namespace BlackMagic {

StraightMode::StraightMode(const DriveModeUtilFunctions& utils): DriveMode(utils) {

}

void StraightMode::setTarget(float targetInches) {
    this->targetInches = targetInches;
}

void StraightMode::run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {

}

bool StraightMode::hasSettled() {
    return false;
}

};
