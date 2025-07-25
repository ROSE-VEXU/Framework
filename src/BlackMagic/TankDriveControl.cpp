#include "vex.h"

namespace BlackMagic {

float TankDriveControl::getLeftSpeed() {
    return mainController.Axis3.position();
}

float TankDriveControl::getRightSpeed() {
    return mainController.Axis2.position();
}

};