#include "vex.h"

namespace BlackMagic {

PipelineMode::PipelineMode(): linear_pid(PID::ZERO_PID), angular_pid(PID::ZERO_PID) {}

void PipelineMode::setTarget(Pose target_pose) {
    if (this->pipeline == nullptr) return;

    this->pipeline->setTarget(target_pose);
}

void PipelineMode::run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) {
    if (this->pipeline == nullptr) return;

    this->pipeline->runPipeline(drive_state, linear_pid, angular_pid);
}

void PipelineMode::setPipeline(std::shared_ptr<AutonomousPipeline> pipeline) {
    this->pipeline = pipeline;
}

bool PipelineMode::hasSettled(const DrivetrainState& drive_state) {
    if (this->pipeline == nullptr) return true;
    return this->pipeline->hasSettled(drive_state);
}

DriveSpeeds PipelineMode::getSpeeds() {
    if (this->pipeline == nullptr) return { 0.0, 0.0 };
    return this->pipeline->getSpeeds();
}

};
