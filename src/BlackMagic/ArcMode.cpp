#include "vex.h"

namespace BlackMagic {

ArcMode::ArcMode(const DriveModeUtilFunctions& utils): DriveMode(utils) {

}

void ArcMode::run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    
}

bool ArcMode::hasSettled() {
    return false;
}

};
