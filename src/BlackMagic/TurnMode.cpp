#include "vex.h"

namespace BlackMagic {

TurnMode::TurnMode() {

}

void TurnMode::setTarget(float targetHeading) {
    this->targetHeading = targetHeading;
}


void TurnMode::run() {
    
}

bool TurnMode::hasSettled() {
    return false;
}

};
