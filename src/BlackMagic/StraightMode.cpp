#include "vex.h"

namespace BlackMagic {

StraightMode::StraightMode() {

}

void StraightMode::setTarget(float targetInches) {
    this->targetInches = targetInches;
}

void StraightMode::run() {
    
}

bool StraightMode::hasSettled() {
    return false;
}

};
