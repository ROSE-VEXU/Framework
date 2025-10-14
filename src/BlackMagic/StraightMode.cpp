#include "vex.h"

namespace BlackMagic {

StraightMode::StraightMode() {

}

void StraightMode::setTarget(float targetInches) {
    this->targetInches = targetInches;
}

void StraightMode::run(std::shared_ptr<PID> pid) {

}

bool StraightMode::hasSettled() {
    return false;
}

};
