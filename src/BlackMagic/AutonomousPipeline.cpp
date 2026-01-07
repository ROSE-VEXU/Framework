#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline() {}

void AutonomousPipeline::setTarget(Pose target_pose) {
    this->target_pose = target_pose;

    if (speedController != nullptr) {
        speedController->updateTarget(this->target_pose);
    }
}

void AutonomousPipeline::setPosition(Position position) {
    if (odometrySource == nullptr) return;
    odometrySource->setPosition(position);
}

int AutonomousPipeline::runPipeline(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    // while(true) {
        if (odometrySource == nullptr) return 1;//continue;
        odometrySource->update();
        odomPosition = odometrySource->getPosition();

        if (localizationSource != nullptr) {
            localizedPosition = localizationSource->getPosition();
        }

        if (speedController == nullptr) return 1;//continue;
        Pose current_pose = { getPosition(), drive_state.heading };
        speedController->update(current_pose, drive_state, linear_pid, angular_pid);

        // vex::wait(VEX_SLEEP_MSEC);
    // }

    return 0;
}

bool AutonomousPipeline::hasSettled(const DrivetrainState& drive_state) {
    if (speedController == nullptr) return true;
    return speedController->hasSettled(drive_state);
}

Position AutonomousPipeline::getPosition() {
    if (localizationSource != nullptr) return localizedPosition;
    return odomPosition;
}

DriveSpeeds AutonomousPipeline::getSpeeds() {
    if (speedController == nullptr) {
        return { 0.0, 0.0 }; // No speed if there's no controller
    }

    DriveSpeeds speeds = speedController->getSpeeds();
    
    return speeds;
}

};