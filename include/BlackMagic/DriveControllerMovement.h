#ifndef DRIVE_CONTROLLER_MOVEMENT_H
#define DRIVE_CONTROLLER_MOVEMENT_H

#include "DriveSpeedProvider.h"

namespace BlackMagic {

class DriveControllerMovement: IDriveSpeedProvider {
public:
    DriveControllerMovement(const vex::controller::axis& first_axis, const vex::controller::axis& second_axis);
    DriveSpeeds getSpeeds() = 0;
protected:
    const vex::controller::axis& first_axis;
    const vex::controller::axis& second_axis;
};

class TankDriveControl: public DriveControllerMovement {
public:
    TankDriveControl(const vex::controller::axis& left_axis, const vex::controller::axis& right_axis);
    DriveSpeeds getSpeeds() override;
};

};

#endif