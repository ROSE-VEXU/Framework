#include "vex.h"

namespace BlackMagic {

Subsystem& Subsystem::onPress(vex::controller::button& button, const std::function<void()> callback) {
    button.pressed([]() {
        // std::move(callback)();
    });
    return *this;
}

Subsystem& Subsystem::onRelease(vex::controller::button& button, const std::function<void()> callback) {
    button.released([]() {
        // std::move(callback);
    });
    return *this;
}

void Subsystem::opControl() {}
    
};
