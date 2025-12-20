#include "vex.h"

namespace BlackMagic {

TankDriveControl::TankDriveControl(const vex::controller& mainController): DriveControllerMovement(mainController) {};

DriveSpeeds TankDriveControl::getSpeeds() {
    return {
        static_cast<float>(mainController.Axis3.position()), // Left
        static_cast<float>(mainController.Axis2.position()) // Right
    };
}

};