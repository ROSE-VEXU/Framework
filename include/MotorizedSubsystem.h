#ifndef CONTROLLED_SUBSYSTEM_H
#define CONTROLLED_SUBSYSTEM_H

#include "PID.h"
#include "Subsystem.h"
#include <memory>
#include <utility>

namespace BlackMagic {

template <typename MotorizedSubclass>
class MotorizedSubsystem: public Subsystem {
public:
    MotorizedSubsystem(): pid(nullptr) {};
    MotorizedSubclass& withPID(PID&& pid) {
        this->pid = std::make_unique<PID>(std::move(pid));
        return static_cast<MotorizedSubclass&>(*this);
    }

protected:
    std::unique_ptr<PID> pid;
};

};

#endif