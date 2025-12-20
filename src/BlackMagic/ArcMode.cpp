#include "vex.h"

namespace BlackMagic {

void ArcMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    
}

bool ArcMode::hasSettled() {
    return false;
}

DriveSpeeds ArcMode::getSpeeds() {
    return { 0.0, 0.0 };
}

};
