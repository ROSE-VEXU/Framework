#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline() {}

void AutonomousPipeline::setTarget(float targetXPosition, float targetYPosition, float targetHeading) {
    this->targetXPosition = targetXPosition;
    this->targetYPosition = targetYPosition;

    if (speedController != nullptr) {
        speedController->updateTarget(targetXPosition, targetYPosition, targetHeading);
    }
}

int AutonomousPipeline::runPipeline() {
    while(true) {
        if (odometrySource == nullptr) continue;
        rawXPosition = odometrySource->getX();
        rawYPosition = odometrySource->getY();

        if (localizationSource != nullptr) {
            localizedXPosition = localizationSource->getX();
            localizedYPosition = localizationSource->getY();
        }

        if (speedController == nullptr) continue;
        speedController->update(getX(), getY(), 0.0);

        vex::wait(VEX_SLEEP_MSEC);
    }

    return 0;
}

float AutonomousPipeline::getX() {
    if (localizationSource != nullptr) return localizedXPosition;
    return rawXPosition;
}

float AutonomousPipeline::getY() {
    if (localizationSource == nullptr) return localizedYPosition;
    return rawYPosition;
}

float AutonomousPipeline::getLeftSpeed() {
    if (speedController == nullptr) {
        return 0.0; // No speed if there's no controller
    }
    return speedController->getLeftSpeed();
}

float AutonomousPipeline::getRightSpeed() {
    if (speedController == nullptr) {
        return 0.0; // No speed if there's no controller
    }
    return speedController->getRightSpeed();
}

};