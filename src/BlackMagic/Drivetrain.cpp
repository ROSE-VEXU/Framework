#include "vex.h"

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, const double& wheelDiameterInches, PID&& pid): MotorizedSubsystem<Drivetrain>(),
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    wheelDiameterInches(wheelDiameterInches),
    kA(0.0),
    driveMode(driveModes[STRAIGHT_MODE]) {
    this->pid = std::make_unique<PID>(std::move(pid));
}

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

Drivetrain&& Drivetrain::withAlignmentCorrection(float alignmentConstant) {
    kA = alignmentConstant;
    return std::move(*this);
}

int Drivetrain::driveTask() {
    while(true) {
        driveMode->run();

        vex::wait(VEX_SLEEP_MSEC);
    }

    return 0;
}

// Private
void Drivetrain::driveLeft(float speedPercent) {
    leftMotors.spin(vex::directionType::fwd, MV(speedPercent), vex::voltageUnits::mV);
}

void Drivetrain::driveRight(float speedPercent) {
    rightMotors.spin(vex::directionType::fwd, MV(speedPercent), vex::voltageUnits::mV);
}

void Drivetrain::driveStraight(float inches) {

}

void Drivetrain::driveStraightAsync() {

}

void Drivetrain::driveTurn(float heading) {

}

void Drivetrain::driveTurnAsync() {

}

void Drivetrain::driveArc(float radius, float degrees, Direction direction) {

}

void Drivetrain::driveArcAsync() {

}

void Drivetrain::driveUsingController(float targetX, float targetY) {

}

void Drivetrain::driveUsingControllerAsync() {

}

bool Drivetrain::hasSettled() {
    return true;
}

void Drivetrain::resetEncoders() {

}

void Drivetrain::stop() {

}

void Drivetrain::setBrake(vex::brakeType brakeMode) {

}

float Drivetrain::getHeading() {
    return 0;
}

float Drivetrain::getLeftDegrees() {
    return 0;
}

float Drivetrain::getRightDegrees() {
    return 0;
}

};
