#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "AutonomousPipeline.h"
#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "PID.h"
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

    Drivetrain& withStraightPID(PID&& pid);
    Drivetrain& withTurnPID(PID&& pid);
    Drivetrain& withArcPID(PID&& pid);

private:
    vex::motor_group& leftMotors;
    vex::motor_group& rightMotors;
    const float wheelDiameterInches;
    std::unique_ptr<DriveControllerMovement> driveControl;
    std::unique_ptr<AutonomousPipeline> autonomousControlPipeline;
    float kA;

    // All 0-value PIDs will lead to no movement, a graceful failure in the unconfigured case.
    std::shared_ptr<PID> drivePIDs[4] = { std::make_shared<PID>(0, 0, 0), std::make_shared<PID>(0, 0, 0), std::make_shared<PID>(0, 0, 0), std::make_shared<PID>(0, 0, 0) };
    std::shared_ptr<DriveMode> driveModes[4] = { std::make_unique<StraightMode>(), std::make_unique<TurnMode>(), std::make_unique<ArcMode>(), std::make_unique<PipelineMode>() };
    int selectedDriveMode;

    void driveLeft(float speedPercent);
    void driveRight(float speedPercent);
    void driveStraight(float inches);
    void driveTurn(float heading);
    void driveArc(float radius, float degrees, Direction direction);
    void driveUsingController(float targetX, float targetY);
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