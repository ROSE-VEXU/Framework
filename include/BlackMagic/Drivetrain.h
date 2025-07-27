#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "AutonomousPipeline.h"
#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "MotorizedSubsystem.h"
#include <algorithm>
#include <memory>

#define STRAIGHT_MODE 0
#define TURN_MODE 1
#define ARC_MODE 2
#define PIPELINE_MODE 3

namespace BlackMagic {

class Drivetrain: public MotorizedSubsystem<Drivetrain> {
public:
    Drivetrain(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, const double& wheelDiameterInches, PID&& pid);

    void opControl();

    template<typename ControllerMovementType>
    Drivetrain&& withControllerMovement(ControllerMovementType&& controllerMovement) {
        VERIFY_SUBCLASS(ControllerMovementType, DriveControllerMovement, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        driveControl = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controllerMovement));
        return std::move(*this);
    };

    Drivetrain&& withAutonomousPipeline(AutonomousPipeline& pipeline);
    Drivetrain&& withAlignmentCorrection(float kA);
    int driveTask();

private:
    vex::motor_group& leftMotors;
    vex::motor_group& rightMotors;
    const float wheelDiameterInches;
    std::unique_ptr<DriveControllerMovement> driveControl;
    std::unique_ptr<AutonomousPipeline> autonomousControlPipeline;
    float kA;
    std::shared_ptr<DriveMode> driveModes[4] = { std::make_unique<StraightMode>(), std::make_unique<TurnMode>(), std::make_unique<ArcMode>(), std::make_unique<PipelineMode>() };
    int selectedDriveMode;

    void driveLeft(float speedPercent);
    void driveRight(float speedPercent);
    void driveStraight(float inches);
    void driveStraightAsync();
    void driveTurn(float heading);
    void driveTurnAsync();
    void driveArc(float radius, float degrees, Direction direction);
    void driveArcAsync();
    void driveUsingController(float targetX, float targetY);
    void driveUsingControllerAsync();
    bool hasSettled();
    void resetEncoders();
    void stop();
    void setBrake(vex::brakeType brakeMode);
    float getHeading();
    float getLeftDegrees();
    float getRightDegrees();
};

};

#endif