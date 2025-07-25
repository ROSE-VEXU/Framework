#ifndef AUTONOMOUS_SELECTOR_H
#define AUTONOMOUS_SELECTOR_H

#include <functional>
#include <vector>

#include "AutonomousRoutine.h"

namespace BlackMagic {

class AutonomousSelector {
public:
    AutonomousSelector();
    AutonomousRoutine getSelectedRoutine();
    void addRoutine(AutonomousRoutine routine);
private:
    std::vector<AutonomousRoutine> routines;
};

};

#endif