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
#define PIPELINE_MODE 2

namespace BlackMagic {

class Drivetrain: public Subsystem {
public:
    Drivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, IHeadingProvider& heading_provider, vex::brakeType brake_mode);

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

    void driveLeft(float speed_percent);
    void driveRight(float speed_percent);
    void preMove();
    void postMove();
    void driveStraight(float inches, float max_speed, PID linear_pid, PID angular_pid);
    void driveStraight(float inches, PID linear_pid, PID angular_pid);
    void driveTurn(Angle heading, float max_speed, PID angular_pid);
    void driveTurn(Angle heading, PID angular_pid);
    void driveArc(float inches, Angle end_angle, float linear_max_speed, float angular_max_speed, PID linear_pid, PID angular_pid);
    void driveArc(float inches, Angle end_angle, PID linear_pid, PID angular_pid);
    void drivePipeline(Pose target_pose, float linear_max_speed, float angular_max_speed, PID linear_pid, PID angular_pid);
    void drivePipeline(Pose target_pose, PID linear_pid, PID angular_pid);
    void enableDriveTask();
    void disableDriveTask();
    int driveTask();
    bool hasSettled();
    void resetEncoders();
    void calibrateHeading();
    void stop();
    void setBrake(vex::brakeType brake_mode);
    void setHeading(float degrees);
    void setPipelinePose(Pose target_pose);
    void setPIDs(PID linear_pid, PID angular_pid);
    DrivetrainState getDriveState();
    float getLeftDegrees();
    float getRightDegrees();
    Angle getHeading();

private:
    vex::motor_group& left_motors;
    vex::motor_group& right_motors;
    IHeadingProvider& heading_provider;
    vex::brakeType brake_mode;
    std::unique_ptr<DriveControllerMovement> drive_control;
    std::shared_ptr<AutonomousPipeline> autonomous_pipeline;

    PID current_linear_pid;
    PID current_angular_pid;

    std::shared_ptr<IDriveMode> drive_modes[3] = { std::make_shared<StraightMode>(), std::make_shared<TurnMode>(), std::make_shared<PipelineMode>() };
    int selected_drive_mode;
    
    bool drive_task_enabled;
};

};

#endif