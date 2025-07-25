#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <functional>

namespace BlackMagic {

class Subsystem {
public:
    Subsystem& onPress(vex::controller::button& button, const std::function<void()> callback);
    Subsystem& onRelease(vex::controller::button& button, const std::function<void()> callback);
    void opControl();
};

};

#endif