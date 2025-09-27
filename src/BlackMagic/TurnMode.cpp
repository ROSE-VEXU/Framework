#include "vex.h"

namespace BlackMagic {

TurnMode::TurnMode() {

}

void TurnMode::setTarget(float targetHeading) {
    this->targetHeading = targetHeading;
}


void TurnMode::run(std::shared_ptr<PID> pid) {
    
}

bool TurnMode::hasSettled() {
    return false;
}

};
