#include "vex.h"

DriveToPoseSpeedController::DriveToPoseSpeedController() {}

void DriveToPoseSpeedController::updateTarget(float positionX, float positionY, float heading) {
    targetPositionX = positionX;
    targetPositionY = positionY;
    targetHeading = heading;
}

void DriveToPoseSpeedController::update(float positionX, float positionY, float heading) {
    
}

float DriveToPoseSpeedController::getLeftSpeed() {
    return /*clamp(*/leftRawSpeed/*)*/;
}

float DriveToPoseSpeedController::getRightSpeed() {
    return /*clamp(*/rightRawSpeed/*)*/;
}
