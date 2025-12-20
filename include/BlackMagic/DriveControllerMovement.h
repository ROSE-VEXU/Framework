#ifndef DRIVE_CONTROLLER_MOVEMENT_H
#define DRIVE_CONTROLLER_MOVEMENT_H

#include "DriveSpeedProvider.h"

namespace BlackMagic {

class DriveControllerMovement: IDriveSpeedProvider {
public:
    DriveControllerMovement(const vex::controller& mainController);
    DriveSpeeds getSpeeds() = 0;
protected:
    const vex::controller& mainController;
};

class TankDriveControl: public DriveControllerMovement {
public:
    TankDriveControl(const vex::controller& mainController);
    DriveSpeeds getSpeeds() override;
};

};

#endif