#include "vex.h"

namespace BlackMagic {

TurnMode::TurnMode() {

}

void TurnMode::setTarget(float targetHeading) {
    this->targetHeading = targetHeading;
}

void TurnMode::run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {

}

bool TurnMode::hasSettled() {
    return false;
}

};
