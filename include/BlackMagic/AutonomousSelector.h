#ifndef AUTONOMOUS_SELECTOR_H
#define AUTONOMOUS_SELECTOR_H

#include <functional>
#include <vector>

#include "AutonomousRoutine.h"

namespace BlackMagic {

class IAutonomousSelector {
public:
    IAutonomousSelector() = default;
    virtual AutonomousRoutine getSelectedRoutine() {
        return AutonomousRoutine{"Default Routine", [](){}};
    }

    virtual void addRoutine(AutonomousRoutine routine) {
        routines.push_back(routine);
    }
protected:
    std::vector<AutonomousRoutine> routines;
};

};

#endif