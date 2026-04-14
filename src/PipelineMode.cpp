#include "vex.h"

namespace BlackMagic {

PipelineMode::PipelineMode() {}

void PipelineMode::setTarget(Pose target_pose) {
    if (this->pipeline == nullptr) return;

    this->pipeline->setTarget(target_pose);
}

void PipelineMode::run(PID& linear_pid, PID& angular_pid) {
    if (this->pipeline == nullptr) return;

    this->pipeline->runPipeline(linear_pid, angular_pid);
}

void PipelineMode::setPipeline(std::shared_ptr<AutonomousPipeline> pipeline) {
    this->pipeline = pipeline;
}

bool PipelineMode::hasSettled() {
    if (this->pipeline == nullptr) return true;
    return this->pipeline->hasSettled();
}

DriveSpeeds PipelineMode::getSpeeds() {
    if (this->pipeline == nullptr) return { 0.0, 0.0 };
    return this->pipeline->getSpeeds();
}

};
