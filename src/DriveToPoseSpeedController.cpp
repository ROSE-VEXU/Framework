#include "vex.h"

DriveToPoseSpeedController::DriveToPoseSpeedController() {}

void DriveToPoseSpeedController::updateTarget(BlackMagic::Position targetPosition, float targetHeading) {
    this->targetPosition = targetPosition;
    this->targetHeading = targetHeading;
}

void DriveToPoseSpeedController::update(BlackMagic::Position currentPosition, float currentHeading) {
    float linearSpeed = 0.0;
    float angularSpeed = 0.0;


    BlackMagic::DriveSpeeds scaledSpeeds = getScaledSpeedsFromMax(linearSpeed, angularSpeed);
    leftSpeed = 5.0;//scaledSpeeds.left;
    rightSpeed = 5.0;//scaledSpeeds.right;
}

BlackMagic::DriveSpeeds DriveToPoseSpeedController::getScaledSpeedsFromMax(float linearSpeed, float angularSpeed) {
    float totalSpeed = fabs(linearSpeed) + fabs(angularSpeed);
    float leftSpeed = linearSpeed - angularSpeed;
    float rightSpeed = linearSpeed + angularSpeed;
    if (totalSpeed > 100.0) return { .left = leftSpeed / totalSpeed, .right = rightSpeed / totalSpeed };
    return { .left = leftSpeed, .right = rightSpeed };
}

BlackMagic::DriveSpeeds DriveToPoseSpeedController::getSpeeds() {
    return { leftSpeed, rightSpeed };
}
