#include "vex.h"

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, vex::inertial&& imu): Subsystem(),
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    imu(imu),
    selectedDriveMode(STRAIGHT_MODE) {}

Drivetrain::Drivetrain(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, vex::inertial& imu): Subsystem(),
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    imu(imu),
    selectedDriveMode(STRAIGHT_MODE) {}


void Drivetrain::opControl() {
    if (driveControl != nullptr) {
        driveLeft(driveControl->getLeftSpeed());
        driveRight(driveControl->getRightSpeed());
    }
}

Drivetrain&& Drivetrain::withAutonomousPipeline(AutonomousPipeline& pipeline) {
    autonomousControlPipeline = std::make_unique<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return std::move(*this);
}

int Drivetrain::driveTask() {
    while(true) {
        driveModes[selectedDriveMode]->run(linearPID, angularPID);
        vex::wait(VEX_SLEEP_MSEC);
    }

    return 0;
}

Drivetrain& Drivetrain::withLinearPID(PID&& pid) {
    this->linearPID = std::make_unique<PID>(std::move(pid));
    return *this;
}

Drivetrain& Drivetrain::withAngularPID(PID&& pid) {
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
    while(!(driveModes[selectedDriveMode]->hasSettled())) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::driveTurn(float heading) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selectedDriveMode = TURN_MODE;
    std::shared_ptr<TurnMode> turnMode = std::static_pointer_cast<TurnMode>(driveModes[selectedDriveMode]);
    turnMode->setTarget(heading);
    while(!(driveModes[selectedDriveMode]->hasSettled())) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::driveArc(float radius, float degrees, Direction direction) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    selectedDriveMode = ARC_MODE;
    std::shared_ptr<ArcMode> arcMode = std::static_pointer_cast<ArcMode>(driveModes[selectedDriveMode]);
    while(!(driveModes[selectedDriveMode]->hasSettled())) vex::wait(VEX_SLEEP_MSEC);
    stop();
}

void Drivetrain::drivePipeline(float targetX, float targetY, float targetHeading) {
    stop();
    resetEncoders();
    setBrake(vex::brakeType::hold);
    if (autonomousControlPipeline != nullptr) {
        selectedDriveMode = PIPELINE_MODE;
        std::shared_ptr<PipelineMode> pipelineMode = std::static_pointer_cast<PipelineMode>(driveModes[selectedDriveMode]);
        while(!(driveModes[selectedDriveMode]->hasSettled())) vex::wait(VEX_SLEEP_MSEC);
    }
    stop();
}

bool Drivetrain::hasSettled() {
    return driveModes[selectedDriveMode]->hasSettled();
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
