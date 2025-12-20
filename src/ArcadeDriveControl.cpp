#include "vex.h"

ArcadeDriveControl::ArcadeDriveControl(const vex::controller& mainController): DriveControllerMovement(mainController) {};

BlackMagic::DriveSpeeds ArcadeDriveControl::getSpeeds() {
    return {
        static_cast<float>(mainController.Axis3.position() + mainController.Axis1.position()), // Left
        static_cast<float>(mainController.Axis3.position() - mainController.Axis1.position()) // Right
    };
}
