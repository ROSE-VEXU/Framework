#ifndef ARCADE_DRIVE_CONTROL_H
#define ARCADE_DRIVE_CONTROL_H

#include "BlackMagic/DriveControllerMovement.h"

class ArcadeDriveControl: public BlackMagic::DriveControllerMovement {
public:
    ArcadeDriveControl(const vex::controller& mainController);
    BlackMagic::DriveSpeeds getSpeeds() override;
};


#endif