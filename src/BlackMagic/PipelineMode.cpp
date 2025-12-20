#include "vex.h"

namespace BlackMagic {

void PipelineMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    if (this->pipeline == nullptr) return;

    this->pipeline->runPipeline();
}

void PipelineMode::setPipeline(std::shared_ptr<AutonomousPipeline> pipeline) {
    this->pipeline = pipeline;
}

bool PipelineMode::hasSettled() {
    return false;
}

DriveSpeeds PipelineMode::getSpeeds() {
    if (this->pipeline == nullptr) return { 0.0, 0.0 };
    return this->pipeline->getSpeeds();
}

};
