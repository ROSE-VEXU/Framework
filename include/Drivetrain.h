#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "AutonomousPipeline.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "DriveStates.h"
#include "PID.h"
#include "PositionProvider.h"
#include "Subsystem.h"
#include <algorithm>
#include <memory>

#define STRAIGHT_MODE 0
#define TURN_MODE 1
#define PIPELINE_MODE 3

namespace BlackMagic {

class Drivetrain: public Subsystem {
public:
    Drivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, IHeadingProvider& heading_provider);

    void opControl();

    template<typename ControllerMovementType>
    Drivetrain&& withControllerMovement(ControllerMovementType&& controller_movement) && {
        VERIFY_SUBCLASS(ControllerMovementType, DriveControllerMovement, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        drive_control = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controller_movement));
        return std::move(*this);
    };
    
    template<typename ControllerMovementType>
    Drivetrain& withControllerMovement(ControllerMovementType&& controller_movement) & {
        VERIFY_SUBCLASS(ControllerMovementType, DriveControllerMovement, "withControllerMovement", "controllerMovement", "DriveControllerMovement");
        drive_control = std::make_unique<std::decay_t<ControllerMovementType>>(std::forward<ControllerMovementType>(controller_movement));
        return *this;
    };

    Drivetrain&& withAutonomousPipeline(AutonomousPipeline&& pipeline) &&;
    Drivetrain& withAutonomousPipeline(AutonomousPipeline&& pipeline) &;

    Drivetrain&& withLinearPID(PID&& pid) &&;
    Drivetrain& withLinearPID(PID&& pid) &;
    Drivetrain&& withAngularPID(PID&& pid) &&;
    Drivetrain& withAngularPID(PID&& pid) &;

    void driveLeft(float speed_percent);
    void driveRight(float speed_percent);
    void driveStraight(float inches);
    void driveTurn(Angle heading);
    void driveArc(float inches, Angle end_angle);
    void drivePipeline(Pose target_pose);
    bool hasSettled();
    void resetEncoders();
    void calibrateHeading();
    void stop();
    void setBrake(vex::brakeType brake_mode);
    void setHeading(float degrees);
    void setPipelinePose(Pose target_pose);
    DrivetrainState getDriveState();
    float getLeftDegrees();
    float getRightDegrees();
    Angle getHeading();

    void enableDriveTask();
    void disableDriveTask();
    int driveTask();

private:
    vex::motor_group& left_motors;
    vex::motor_group& right_motors;
    IHeadingProvider& heading_provider;
    std::unique_ptr<DriveControllerMovement> drive_control;
    std::shared_ptr<AutonomousPipeline> autonomous_pipeline;

    // All 0-value PIDs will lead to no movement, a graceful failure in the unconfigured case.
    std::shared_ptr<PID> linear_pid = std::make_shared<PID>(0, IntegralConfig{0, 0, 0}, 0);
    std::shared_ptr<PID> angular_pid = std::make_shared<PID>(0, IntegralConfig{0, 0, 0}, 0);
    std::shared_ptr<IDriveMode> drive_modes[3] = { std::make_shared<StraightMode>(), std::make_shared<TurnMode>(), std::make_shared<PipelineMode>() };
    int selected_drive_mode;

    bool drive_task_enabled;
};

};

#endif