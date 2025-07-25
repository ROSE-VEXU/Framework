#ifndef CONTROLLED_SUBSYSTEM_H
#define CONTROLLED_SUBSYSTEM_H

#include "PID.h"
#include "Subsystem.h"

namespace BlackMagic {

class ControlledSubsystem: public Subsystem {
public:
    ControlledSubsystem(PID pid);

private:
    PID pid;
};

};

#endif