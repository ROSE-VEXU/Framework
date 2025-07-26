#include "vex.h"

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(): MotorizedSubsystem<Drivetrain>(),
  //   driveControl(new TankDriveControl()),
                                    //   autonomousControlPipeline(),
                                    kA(0.0),
                                    driveMode(driveModes[STRAIGHT_MODE]) {
}

void Drivetrain::opControl() {
    // TODO - set left & right speeds
}

Drivetrain&& Drivetrain::withAutonomousPipeline(AutonomousPipeline& pipeline) {
    autonomousControlPipeline = std::make_unique<BlackMagic::AutonomousPipeline>(std::move(pipeline));
    return std::move(*this);
}

Drivetrain&& Drivetrain::withAlignmentCorrection(float alignmentConstant) {
    kA = alignmentConstant;
    return std::move(*this);
}

int driveTask() {
    return 0;
}

// Private
void Drivetrain::driveLeft(float speedPercent) {

}

void Drivetrain::driveRight(float speedPercent) {

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
