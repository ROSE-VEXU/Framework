#include "vex.h"

namespace BlackMagic {

void StraightMode::setTarget(float targetInches) {
    this->targetInches = targetInches;
}

void StraightMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {

}

bool StraightMode::hasSettled() {
    return false;
}

};
