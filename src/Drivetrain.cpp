#include "vex.h"
#include <cstddef>
#include <memory>

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, IHeadingProvider& heading_provider): Subsystem(),
    left_motors(left_motors),
    right_motors(right_motors),
    heading_provider(heading_provider),
    selected_drive_mode(STRAIGHT_MODE),
    drive_task_enabled(false) {
}

void Drivetrain::opControl() {
    if (driveControl != nullptr && !drive_task_enabled) {
        setBrake(vex::brakeType::coast);
        DriveSpeeds speeds = driveControl->getSpeeds();
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

Drivetrain&& Drivetrain::withLinearPID(PID&& pid) && {
    this->linear_pid = std::make_unique<PID>(std::move(pid));
    return std::move(*this);
}

Drivetrain& Drivetrain::withLinearPID(PID&& pid) & {
    this->linear_pid = std::make_unique<PID>(std::move(pid));
    return *this;
}

Drivetrain&& Drivetrain::withAngularPID(PID&& pid) && {
    this->angular_pid = std::make_unique<PID>(std::move(pid));
    return std::move(*this);
}

Drivetrain& Drivetrain::withAngularPID(PID&& pid) & {
    this->angular_pid = std::make_unique<PID>(std::move(pid));
    return *this;
}


// Private
void Drivetrain::driveLeft(float speed_percent) {
    left_motors.spin(vex::directionType::fwd, MV(speed_percent), vex::voltageUnits::mV);
}

void Drivetrain::driveRight(float speed_percent) {
    right_motors.spin(vex::directionType::fwd, MV(speed_percent), vex::voltageUnits::mV);
}

void Drivetrain::driveStraight(float inches) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    linear_pid->reset();
    angular_pid->reset();
    selected_drive_mode = STRAIGHT_MODE;
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[selected_drive_mode]);
    straight_mode->setTarget(inches, getHeading());
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveTurn(Angle heading) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    angular_pid->reset();
    selected_drive_mode = TURN_MODE;
    std::shared_ptr<TurnMode> turn_mode = std::static_pointer_cast<TurnMode>(drive_modes[selected_drive_mode]);
    turn_mode->setTarget(heading);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveArc(float inches, Angle end_angle) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    linear_pid->reset();
    angular_pid->reset();
    selected_drive_mode = STRAIGHT_MODE;
    std::shared_ptr<StraightMode> straight_mode = std::static_pointer_cast<StraightMode>(drive_modes[selected_drive_mode]);
    straight_mode->setTarget(inches, end_angle);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    linear_pid->reset();
    angular_pid->reset();
    if (autonomous_pipeline != nullptr) {
        selected_drive_mode = PIPELINE_MODE;
        std::shared_ptr<PipelineMode> pipeline_mode = std::static_pointer_cast<PipelineMode>(drive_modes[selected_drive_mode]);
        pipeline_mode->setPipeline(autonomous_pipeline);
        autonomous_pipeline->setTarget(target_pose);
        while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    }
    stop();
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

void Drivetrain::enableDriveTask() {
    drive_task_enabled = true;
}

void Drivetrain::disableDriveTask() {
    drive_task_enabled = false;
}

int Drivetrain::driveTask() {
    while(drive_task_enabled) {
        drive_modes[selected_drive_mode]->run(getDriveState(), linear_pid, angular_pid);
        DriveSpeeds speeds = drive_modes[selected_drive_mode]->getSpeeds();
        driveLeft(speeds.left);
        driveRight(speeds.right);

        vex::wait(VEX_SLEEP_MSEC_SHORT);
    }

    return 0;
}

};
