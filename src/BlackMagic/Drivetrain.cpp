#include "vex.h"

namespace BlackMagic {

// Public
Drivetrain::Drivetrain(PID pid): ControlledSubsystem(pid),
  //   driveControl(new TankDriveControl()),
                                    //   autonomousControlPipeline(),
                                    kA(0.0),
                                    selectedDriveMode(STRAIGHT_MODE) {
}

void Drivetrain::opControl() {
    
}

Drivetrain& Drivetrain::withControllerMovement(DriveControllerMovement controllerMovement) {
    // driveControl = std::make_unique<BlackMagic::DriveControllerMovement>(std::move(controllerMovement));
    return *this;
}

Drivetrain& Drivetrain::withAutonomousPipeline(AutonomousPipeline pipeline) {
    // autonomousControlPipeline = std::move(pipeline);
    return *this;
}

Drivetrain& Drivetrain::withAlignmentCorrection(float alignmentConstant) {
    kA = alignmentConstant;
    return *this;
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
