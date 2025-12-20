#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline() {}

void AutonomousPipeline::setTarget(Position targetPosition, float targetHeading) {
    this->targetPosition = targetPosition;

    if (speedController != nullptr) {
        speedController->updateTarget(targetPosition, targetHeading);
    }
}

int AutonomousPipeline::runPipeline() {
    // while(true) {
        if (odometrySource == nullptr) return 1;//continue;
        odomPosition = odometrySource->getPosition();

        if (localizationSource != nullptr) {
            localizedPosition = localizationSource->getPosition();
        }

        if (speedController == nullptr) return 1;//continue;
        speedController->update(getPosition(), 0.0);

        // vex::wait(VEX_SLEEP_MSEC);
    // }

    return 0;
}

Position AutonomousPipeline::getPosition() {
    if (localizationSource != nullptr) return localizedPosition;
    return odomPosition;
}

DriveSpeeds AutonomousPipeline::getSpeeds() {
    if (speedController == nullptr) {
        return { 0.0, 0.0 }; // No speed if there's no controller
    }
    return speedController->getSpeeds();
}

};