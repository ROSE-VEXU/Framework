#ifndef AUTONOMOUS_ROUTINE_H
#define AUTONOMOUS_ROUTINE_H

#include <functional>
#include <string>

namespace BlackMagic {

struct AutonomousRoutine {
    std::string name;
    std::function<void()> routine;
};

};

#endif