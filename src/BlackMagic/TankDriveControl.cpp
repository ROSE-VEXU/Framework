#include "vex.h"

namespace BlackMagic {

TankDriveControl::TankDriveControl(const vex::controller::axis& left_axis, const vex::controller::axis& right_axis): DriveControllerMovement(left_axis, right_axis) {};

DriveSpeeds TankDriveControl::getSpeeds() {
    return {
        static_cast<float>(first_axis.position()), // Left
        static_cast<float>(second_axis.position()) // Right
    };
}

};