#include "vex.h"
#include <cstddef>
#include <memory>

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, IHeadingProvider& heading_provider): Subsystem(),
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    heading_provider(heading_provider),
    selectedDriveMode(STRAIGHT_MODE),
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
    autonomousControlPipeline = std::make_shared<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return std::move(*this);
}

Drivetrain& Drivetrain::withAutonomousPipeline(AutonomousPipeline&& pipeline) & {
    autonomousControlPipeline = std::make_shared<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return *this;
}

Drivetrain&& Drivetrain::withLinearPID(PID&& pid) && {
    this->linearPID = std::make_unique<PID>(std::move(pid));
    return std::move(*this);
}

Drivetrain& Drivetrain::withLinearPID(PID&& pid) & {
    this->linearPID = std::make_unique<PID>(std::move(pid));
    return *this;
}

Drivetrain&& Drivetrain::withAngularPID(PID&& pid) && {
    this->angularPID = std::make_unique<PID>(std::move(pid));
    return std::move(*this);
}

Drivetrain& Drivetrain::withAngularPID(PID&& pid) & {
    this->angularPID = std::make_unique<PID>(std::move(pid));
    return *this;
}


// Private
void Drivetrain::driveLeft(float speedPercent) {
    leftMotors.spin(vex::directionType::fwd, MV(speedPercent), vex::voltageUnits::mV);
}

void Drivetrain::driveRight(float speedPercent) {
    rightMotors.spin(vex::directionType::fwd, MV(speedPercent), vex::voltageUnits::mV);
}

void Drivetrain::driveStraight(float inches) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    linearPID->reset();
    angularPID->reset();
    selectedDriveMode = STRAIGHT_MODE;
    std::shared_ptr<StraightMode> straightMode = std::static_pointer_cast<StraightMode>(driveModes[selectedDriveMode]);
    straightMode->setTarget(inches, getHeading());
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveTurn(Angle heading) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    angularPID->reset();
    selectedDriveMode = TURN_MODE;
    std::shared_ptr<TurnMode> turnMode = std::static_pointer_cast<TurnMode>(driveModes[selectedDriveMode]);
    turnMode->setTarget(heading);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::driveArc(float radius, float degrees, Direction direction) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selectedDriveMode = ARC_MODE;
    std::shared_ptr<ArcMode> arcMode = std::static_pointer_cast<ArcMode>(driveModes[selectedDriveMode]);
    // TODO - set arc target
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    stop();
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    linearPID->reset();
    angularPID->reset();
    if (autonomousControlPipeline != nullptr) {
        selectedDriveMode = PIPELINE_MODE;
        std::shared_ptr<PipelineMode> pipelineMode = std::static_pointer_cast<PipelineMode>(driveModes[selectedDriveMode]);
        pipelineMode->setPipeline(autonomousControlPipeline);
        autonomousControlPipeline->setTarget(target_pose);
        while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC_SHORT);
    }
    stop();
}

bool Drivetrain::hasSettled() {
    return driveModes[selectedDriveMode]->hasSettled(getDriveState());
}

void Drivetrain::resetEncoders() {
    leftMotors.resetPosition();
    rightMotors.resetPosition();
}

void Drivetrain::stop() {
    leftMotors.stop();
    rightMotors.stop();
}

void Drivetrain::setBrake(vex::brakeType brakeMode) {
    leftMotors.setStopping(brakeMode);
    rightMotors.setStopping(brakeMode);
}

DrivetrainState Drivetrain::getDriveState(){
    return { getLeftDegrees(), getRightDegrees(), getHeading() };
}

float Drivetrain::getLeftDegrees() {
    return leftMotors.position(vex::rotationUnits::deg);
}

float Drivetrain::getRightDegrees() {
    return rightMotors.position(vex::rotationUnits::deg);
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
        // printf("Driving!\n");
        // Utils::robot_brain().Screen.printAt(12, 24, "mode: %d", selectedDriveMode);
        driveModes[selectedDriveMode]->run(getDriveState(), linearPID, angularPID);
        DriveSpeeds speeds = driveModes[selectedDriveMode]->getSpeeds();
        driveLeft(speeds.left);
        driveRight(speeds.right);

        vex::wait(VEX_SLEEP_MSEC_SHORT);
    }

    return 0;
}

};
