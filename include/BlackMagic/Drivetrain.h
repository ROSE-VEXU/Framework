#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "AutonomousPipeline.h"
#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "ControlledSubsystem.h"
#include <memory>

#define STRAIGHT_MODE 0
#define TURN_MODE 1
#define ARC_MODE 2
#define PIPELINE_MODE 3

namespace BlackMagic {

class Drivetrain: public ControlledSubsystem {
public:
    Drivetrain(PID pid);
    void opControl();
    Drivetrain& withControllerMovement(DriveControllerMovement controllerMovement);
    Drivetrain& withAutonomousPipeline(AutonomousPipeline pipeline);
    Drivetrain& withAlignmentCorrection(float kA);
    int driveTask();

private:
    // std::unique_ptr<DriveControllerMovement> driveControl;
    // std::unique_ptr<AutonomousPipeline> autonomousControlPipeline;
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