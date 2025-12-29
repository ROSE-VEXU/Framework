#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline() {}

void AutonomousPipeline::setTarget(Pose target_pose) {
    this->target_pose = target_pose;

    if (speedController != nullptr) {
        speedController->updateTarget(target_pose);
    }
}

int AutonomousPipeline::runPipeline(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    // while(true) {
        if (odometrySource == nullptr) return 1;//continue;
        odomPosition = odometrySource->getPosition();

        if (localizationSource != nullptr) {
            localizedPosition = localizationSource->getPosition();
        }

        if (speedController == nullptr) return 1;//continue;
        speedController->update({ getPosition(), 0.0 }, utils, linear_pid, angular_pid);

        // vex::wait(VEX_SLEEP_MSEC);
    // }

    return 0;
}

bool AutonomousPipeline::hasSettled(const DriveModeUtilFunctions& utils) {
    if (speedController == nullptr) return true;
    return speedController->hasSettled(utils);
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