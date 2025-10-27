#include "vex.h"

namespace BlackMagic {

PipelineMode::PipelineMode(const DriveModeUtilFunctions& utils): DriveMode(utils) {

}

void PipelineMode::run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    
}

void PipelineMode::run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid, AutonomousPipeline pipeline) {
    
}

bool PipelineMode::hasSettled() {
    return false;
}

};
