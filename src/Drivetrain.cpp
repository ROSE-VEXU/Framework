#include "vex.h"
#include <cstddef>
#include <memory>

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, IHeadingProvider& heading_provider): Subsystem(),
    left_motors(left_motors),
    right_motors(right_motors),
    heading_provider(heading_provider),
    current_linear_pid(PID::ZERO_PID),
    current_angular_pid(PID::ZERO_PID),
    selected_drive_mode(STRAIGHT_MODE),
    drive_task_enabled(false) {
}

void Drivetrain::opControl() {
    if (drive_control != nullptr && !drive_task_enabled) {
        setBrake(vex::brakeType::coast);
        DriveSpeeds speeds = drive_control->getSpeeds();
        driveLeft(speeds.left);
        driveRight(speeds.right);
    }
}

Drivetrain&& Drivetrain::withAutonomousPipeline(AutonomousPipeline&& pipeline) && {
    autonomous_pipeline = std::make_shared<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return std::move(*this);
}

Drivetrain& Drivetrain::withAutonomousPipeline(AutonomousPipeline&& pipeline) & {
    autonomous_pipeline = std::make_shared<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return *this;
}



// Private
void Drivetrain::driveLeft(float speed_percent) {
    left_motors.spin(vex::directionType::fwd, MV(speed_percent), vex::voltageUnits::mV);
}

void Drivetrain::driveRight(float speed_percent) {
    right_motors.spin(vex::directionType::fwd, MV(speed_percent), vex::voltageUnits::mV);
}

void Drivetrain::preMove() {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
}

void Drivetrain::postMove() {
    stop();
}

void Drivetrain::driveStraight(float inches, float max_speed, PID linear_pid, PID angular_pid) {
    preMove();
    
    linear_pid.setMaxSpeed(max_speed);
    angular_pid.setMaxSpeed(max_speed);
    setPIDs(linear_pid, angular_pid);
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[STRAIGHT_MODE]);
    straight_mode->setTarget(inches, getHeading());
    selected_drive_mode = STRAIGHT_MODE;
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    
    postMove();
}

void Drivetrain::driveStraight(float inches, PID linear_pid, PID angular_pid) {
    driveStraight(inches, 100.0, linear_pid, angular_pid);
}

void Drivetrain::driveTurn(Angle heading, float max_speed, PID angular_pid) {
    preMove();

    angular_pid.setMaxSpeed(max_speed);
    setPIDs(PID::ZERO_PID, angular_pid);
    std::shared_ptr<TurnMode> turn_mode = std::static_pointer_cast<TurnMode>(drive_modes[TURN_MODE]);
    turn_mode->setTarget(heading);
    selected_drive_mode = TURN_MODE;
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);

    postMove();
}

void Drivetrain::driveTurn(Angle heading, PID angular_pid) {
    driveTurn(heading, 100.0, angular_pid);
}

void Drivetrain::driveArc(float inches, Angle end_angle, float linear_max_speed, float angular_max_speed, PID linear_pid, PID angular_pid) {
    preMove();

    linear_pid.setMaxSpeed(linear_max_speed);
    angular_pid.setMaxSpeed(angular_max_speed);
    setPIDs(linear_pid, angular_pid);
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[STRAIGHT_MODE]);
    straight_mode->setTarget(inches, end_angle);
    selected_drive_mode = STRAIGHT_MODE;
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);

    postMove();
}

void Drivetrain::driveArc(float inches, Angle end_angle, PID linear_pid, PID angular_pid) {
    driveArc(inches, end_angle, 100.0, 100.0, linear_pid, angular_pid);
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose, float linear_max_speed, float angular_max_speed, PID linear_pid, PID angular_pid) {
    preMove();

    if (autonomous_pipeline != nullptr) {
        linear_pid.setMaxSpeed(linear_max_speed);
        angular_pid.setMaxSpeed(angular_max_speed);
        setPIDs(linear_pid, angular_pid);
        std::shared_ptr<PipelineMode> pipeline_mode = std::static_pointer_cast<PipelineMode>(drive_modes[PIPELINE_MODE]);
        pipeline_mode->setPipeline(autonomous_pipeline);
        pipeline_mode->setTarget(target_pose);
        selected_drive_mode = PIPELINE_MODE;
        while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    }

    postMove();
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose, PID linear_pid, PID angular_pid) {
    drivePipeline(target_pose, 100.0, 100.0, linear_pid, angular_pid);
}

void Drivetrain::enableDriveTask() {
    drive_task_enabled = true;
}

void Drivetrain::disableDriveTask() {
    drive_task_enabled = false;
}

int Drivetrain::driveTask() {
    while(drive_task_enabled) {
        drive_modes[selected_drive_mode]->run(getDriveState(), current_linear_pid, current_angular_pid);
        DriveSpeeds speeds = drive_modes[selected_drive_mode]->getSpeeds();

        driveLeft(speeds.left);
        driveRight(speeds.right);

        vex::wait(VEX_SLEEP_MSEC_SHORT);
    }

    return 0;
}

bool Drivetrain::hasSettled() {
    return drive_modes[selected_drive_mode]->hasSettled(getDriveState());
}

void Drivetrain::resetEncoders() {
    left_motors.resetPosition();
    right_motors.resetPosition();
}

void Drivetrain::calibrateHeading() {
    heading_provider.calibrate();
}

void Drivetrain::stop() {
    left_motors.stop();
    right_motors.stop();
}

void Drivetrain::setBrake(vex::brakeType brake_mode) {
    left_motors.setStopping(brake_mode);
    right_motors.setStopping(brake_mode);
}

void Drivetrain::setHeading(float degrees) {
    heading_provider.setHeading(degrees);
}

void Drivetrain::setPipelinePose(Pose target_pose) {
    if (autonomous_pipeline != nullptr) {
        autonomous_pipeline->setPosition(target_pose.position);
    }
    setHeading(target_pose.heading);
}

void Drivetrain::setPIDs(PID linear_pid, PID angular_pid) {
    current_linear_pid = linear_pid;
    current_angular_pid = angular_pid;
}

DrivetrainState Drivetrain::getDriveState(){
    return { getLeftDegrees(), getRightDegrees(), getHeading() };
}

float Drivetrain::getLeftDegrees() {
    return left_motors.position(vex::rotationUnits::deg);
}

float Drivetrain::getRightDegrees() {
    return right_motors.position(vex::rotationUnits::deg);
}

Angle Drivetrain::getHeading() {
    return heading_provider.getHeading();
}

};
