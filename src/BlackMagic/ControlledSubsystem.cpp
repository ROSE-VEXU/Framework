#include "vex.h"

namespace BlackMagic {

ControlledSubsystem::ControlledSubsystem(PID pid): pid(std::move(pid)) {}
    
};
