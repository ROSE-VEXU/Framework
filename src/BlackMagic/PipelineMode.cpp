#include "vex.h"

namespace BlackMagic {

void PipelineMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    
}

void PipelineMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid, AutonomousPipeline pipeline) {
    
}

bool PipelineMode::hasSettled() {
    return false;
}

};
