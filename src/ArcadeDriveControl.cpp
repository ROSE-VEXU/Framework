#include "vex.h"

ArcadeDriveControl::ArcadeDriveControl(const vex::controller::axis& straight_axis, const vex::controller::axis& turn_axis): DriveControllerMovement(straight_axis, turn_axis) {};

BlackMagic::DriveSpeeds ArcadeDriveControl::getSpeeds() {
    return {
        static_cast<float>(first_axis.position() + second_axis.position()), // Left
        static_cast<float>(first_axis.position() - second_axis.position()) // Right
    };
}
