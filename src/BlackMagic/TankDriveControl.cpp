#include "vex.h"

namespace BlackMagic {

TankDriveControl::TankDriveControl(const vex::controller& mainController): DriveControllerMovement(mainController) {};

float TankDriveControl::getLeftSpeed() {
    return mainController.Axis3.position();
}

float TankDriveControl::getRightSpeed() {
    return mainController.Axis2.position();
}

};