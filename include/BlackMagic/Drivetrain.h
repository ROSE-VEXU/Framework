#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "AutonomousPipeline.h"
#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "PID.h"
#include "PositionProvider.h"
#include "Subsystem.h"
#include <algorithm>
#include <memory>

#define STRAIGHT_MODE 0
#define TURN_MODE 1
#define ARC_MODE 2
#define PIPELINE_MODE 3

namespace BlackMagic {

class Drivetrain: public Subsystem {
public:
    Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::inertial& imu);

    void opControl();

    template<typename ControllerMovementType>
    Drivetrain&& withControllerMovement(ControllerMovementType&& controllerMovement) && {
        VERIFY_SUBCLASS(ControllerMovementType, DriveControllerMovement, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        driveControl = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controllerMovement));
        return std::move(*this);
    };
    
    template<typename ControllerMovementType>
    Drivetrain& withControllerMovement(ControllerMovementType&& controllerMovement) & {
        VERIFY_SUBCLASS(ControllerMovementType, DriveControllerMovement, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        driveControl = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controllerMovement));
        return *this;
    };

    Drivetrain&& withAutonomousPipeline(AutonomousPipeline&& pipeline) &&;
    Drivetrain& withAutonomousPipeline(AutonomousPipeline&& pipeline) &;

    Drivetrain&& withLinearPID(PID&& pid) &&;
    Drivetrain& withLinearPID(PID&& pid) &;
    Drivetrain&& withAngularPID(PID&& pid) &&;
    Drivetrain& withAngularPID(PID&& pid) &;

    void driveLeft(float speedPercent);
    void driveRight(float speedPercent);
    void driveStraight(float inches);
    void driveTurn(float heading);
    void driveArc(float radius, float degrees, Direction direction);
    void drivePipeline(Pose target_pose);
    bool hasSettled();
    void resetEncoders();
    void stop();
    void setBrake(vex::brakeType brakeMode);
    float getHeading();
    float getLeftDegrees();
    float getRightDegrees();

    void enableDriveTask();
    void disableDriveTask();
    int driveTask();

private:
    vex::motor_group& leftMotors;
    vex::motor_group& rightMotors;
    vex::inertial& imu;
    std::unique_ptr<DriveControllerMovement> driveControl;
    std::shared_ptr<AutonomousPipeline> autonomousControlPipeline;

    // All 0-value PIDs will lead to no movement, a graceful failure in the unconfigured case.
    const DriveModeUtilFunctions utils = {
        .getLeftDegrees = [this]() -> float { return this->getLeftDegrees(); }, 
        .getRightDegrees = [this]() -> float { return this->getRightDegrees(); }, 
        .getHeading = [this]() -> float { return this->getHeading(); }
    };
    std::shared_ptr<PID> linearPID = std::make_shared<PID>(0, 0, 0);
    std::shared_ptr<PID> angularPID = std::make_shared<PID>(0, 0, 0);
    std::shared_ptr<IDriveMode> driveModes[4] = { std::make_shared<StraightMode>(), std::make_shared<TurnMode>(), std::make_shared<ArcMode>(), std::make_shared<PipelineMode>() };
    int selectedDriveMode;

    bool drive_task_enabled;
};

};

#endif