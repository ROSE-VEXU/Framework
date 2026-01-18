#include "vex.h"
#include <cstddef>
#include <memory>

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, IHeadingProvider& heading_provider): Subsystem(),
    left_motors(left_motors),
    right_motors(right_motors),
    heading_provider(heading_provider),
    linear_pid(PID::ZERO_PID),
    angular_pid(PID::ZERO_PID),
    max_speed(100.0f),
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

void Drivetrain::driveStraight(float inches, float max_speed, PID& linear_pid, PID& angular_pid) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selected_drive_mode = STRAIGHT_MODE;
    setPIDs(linear_pid, angular_pid);
    setMaxSpeed(max_speed);
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[selected_drive_mode]);
    straight_mode->setTarget(inches, getHeading());
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveStraight(float inches, PID& linear_pid, PID& angular_pid) {
    driveStraight(inches, 100.0, linear_pid, angular_pid);
}

void Drivetrain::driveTurn(Angle heading, float max_speed, PID& angular_pid) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selected_drive_mode = TURN_MODE;
    setPIDs(PID::ZERO_PID, angular_pid);
    setMaxSpeed(max_speed);
    std::shared_ptr<TurnMode> turn_mode = std::static_pointer_cast<TurnMode>(drive_modes[selected_drive_mode]);
    turn_mode->setTarget(heading);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveTurn(Angle heading, PID& angular_pid) {
    driveTurn(heading, 100.0, angular_pid);
}

void Drivetrain::driveArc(float inches, Angle end_angle, float max_speed, PID& linear_pid, PID& angular_pid) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selected_drive_mode = STRAIGHT_MODE;
    setPIDs(linear_pid, angular_pid);
    setMaxSpeed(max_speed);
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[selected_drive_mode]);
    straight_mode->setTarget(inches, end_angle);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveArc(float inches, Angle end_angle, PID& linear_pid, PID& angular_pid) {
    driveArc(inches, end_angle, 100.0, linear_pid, angular_pid);
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose, float max_speed, PID& linear_pid, PID& angular_pid) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    if (autonomous_pipeline != nullptr) {
        selected_drive_mode = PIPELINE_MODE;
        setPIDs(linear_pid, angular_pid);
        setMaxSpeed(max_speed);
        std::shared_ptr<PipelineMode> pipeline_mode = std::static_pointer_cast<PipelineMode>(drive_modes[selected_drive_mode]);
        pipeline_mode->setPipeline(autonomous_pipeline);
        pipeline_mode->setTarget(target_pose);
        while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    }
    stop();
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose, PID& linear_pid, PID& angular_pid) {
    drivePipeline(target_pose, 100.0, linear_pid, angular_pid);
}

void Drivetrain::enableDriveTask() {
    drive_task_enabled = true;
}

void Drivetrain::disableDriveTask() {
    drive_task_enabled = false;
}

int Drivetrain::driveTask() {
    while(drive_task_enabled) {
        drive_modes[selected_drive_mode]->run(getDriveState());
        DriveSpeeds speeds = drive_modes[selected_drive_mode]->getSpeeds();
        speeds = { BlackMagic::Utils::clamp(speeds.left, -max_speed, max_speed), BlackMagic::Utils::clamp(speeds.right, -max_speed, max_speed) };

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

void setPIDs(PID& linear_pid, PID& angular_pid) {
    current_linear_pid = linear_pid;
    current_angular_pid = angular_pid;
}

void setMaxSpeed(float max_speed) {
    this->max_speed = max_speed;
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
