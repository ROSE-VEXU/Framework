#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline() {}

void AutonomousPipeline::setTarget(Pose target_pose) {
    this->target_pose = target_pose;

    if (speed_controller != nullptr) {
        speed_controller->updateTarget(this->target_pose);
    }
}

void AutonomousPipeline::setPosition(Position position) {
    if (odometry_source == nullptr) return;
    odometry_source->setPosition(position);
    odom_position = position;
}

int AutonomousPipeline::runPipeline(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    // while(true) {
        if (odometry_source == nullptr) return 1;//continue;
        odometry_source->update();
        odom_position = odometry_source->getPosition();

        if (localization_source != nullptr) {
            localized_position = localization_source->getPosition();
        }

        if (speed_controller == nullptr) return 1;//continue;
        Pose current_pose = { getPosition(), drive_state.heading };
        speed_controller->update(current_pose, drive_state, linear_pid, angular_pid);

        // vex::wait(VEX_SLEEP_MSEC);
    // }

    return 0;
}

bool AutonomousPipeline::hasSettled(const DrivetrainState& drive_state) {
    if (speed_controller == nullptr) return true;
    return speed_controller->hasSettled(drive_state);
}

Position AutonomousPipeline::getPosition() {
    if (localization_source != nullptr) return localized_position;
    return odom_position;
}

DriveSpeeds AutonomousPipeline::getSpeeds() {
    if (speed_controller == nullptr) {
        return { 0.0, 0.0 }; // No speed if there's no controller
    }

    DriveSpeeds speeds = speed_controller->getSpeeds();
    
    return speeds;
}

};