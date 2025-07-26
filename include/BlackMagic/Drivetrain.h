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
    Drivetrain();

    void opControl();

    template<typename ControllerMovementType>
    Drivetrain&& withControllerMovement(ControllerMovementType&& controllerMovement) {
        VERIFY_SUBCLASS(DriveControllerMovement, ControllerMovementType, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        driveControl = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controllerMovement));
        return std::move(*this);
    };

    Drivetrain&& withAutonomousPipeline(AutonomousPipeline& pipeline);
    Drivetrain&& withAlignmentCorrection(float kA);
    int driveTask();

private:
    std::unique_ptr<DriveControllerMovement> driveControl;
    std::unique_ptr<AutonomousPipeline> autonomousControlPipeline;
    float kA;
    DriveMode* driveModes[4] = { new StraightMode(), new TurnMode(), new ArcMode(), new PipelineMode() };
    DriveMode* driveMode;

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