#include "vex.h"
#include <cstddef>

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::inertial& imu): Subsystem(),
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    imu(imu),
    selectedDriveMode(STRAIGHT_MODE) {}

void Drivetrain::opControl() {
    if (driveControl != nullptr) {
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


int Drivetrain::driveTask() {
    while(true) {
        driveModes[selectedDriveMode]->run(utils, linearPID, angularPID);
        DriveSpeeds speeds = driveModes[selectedDriveMode]->getSpeeds();
        driveLeft(speeds.left);
        driveRight(speeds.right);
        vex::wait(VEX_SLEEP_MSEC);
    }

    return 0;
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
    selectedDriveMode = STRAIGHT_MODE;
    std::shared_ptr<StraightMode> straightMode = std::static_pointer_cast<StraightMode>(driveModes[selectedDriveMode]);
    straightMode->setTarget(inches);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::driveTurn(float heading) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selectedDriveMode = TURN_MODE;
    std::shared_ptr<TurnMode> turnMode = std::static_pointer_cast<TurnMode>(driveModes[selectedDriveMode]);
    turnMode->setTarget(heading);
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::driveArc(float radius, float degrees, Direction direction) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selectedDriveMode = ARC_MODE;
    std::shared_ptr<ArcMode> arcMode = std::static_pointer_cast<ArcMode>(driveModes[selectedDriveMode]);
    // TODO - set arc target
    while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::drivePipeline(BlackMagic::Pose target_pose) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    if (autonomousControlPipeline != nullptr) {
        selectedDriveMode = PIPELINE_MODE;
        std::shared_ptr<PipelineMode> pipelineMode = std::static_pointer_cast<PipelineMode>(driveModes[selectedDriveMode]);
        pipelineMode->setPipeline(autonomousControlPipeline);
        autonomousControlPipeline->setTarget(target_pose);
        while(!hasSettled()) vex::wait(VEX_SLEEP_MSEC);
    }
    stop();
}

bool Drivetrain::hasSettled() {
    return driveModes[selectedDriveMode]->hasSettled(utils);
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

float Drivetrain::getHeading() {
    return imu.heading(vex::rotationUnits::deg);
}

float Drivetrain::getLeftDegrees() {
    return leftMotors.position(vex::rotationUnits::deg);
}

float Drivetrain::getRightDegrees() {
    return rightMotors.position(vex::rotationUnits::deg);
}

};
