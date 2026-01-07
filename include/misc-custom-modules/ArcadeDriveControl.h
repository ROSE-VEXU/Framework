#ifndef ARCADE_DRIVE_CONTROL_H
#define ARCADE_DRIVE_CONTROL_H

class ArcadeDriveControl: public BlackMagic::DriveControllerMovement {
public:
    ArcadeDriveControl(const vex::controller::axis& straight_axis, const vex::controller::axis& turn_axis);
    BlackMagic::DriveSpeeds getSpeeds() override;
};


#endif