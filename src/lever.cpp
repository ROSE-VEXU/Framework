#include "vex.h"

vex::motor leftLever = vex::motor(vex::PORT9, true);
vex::motor rightLever = vex::motor(vex::PORT10, false);

Lever::Lever(const vex::controller::button& up_button, const vex::controller::button& down_button) : 
    up_button(up_button), down_button(down_button) {
}

void Lever::opControl() {
    float speed = getLeverSpeed(getLeverMode());
    leftLever.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    rightLever.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

Lever::LeverMode Lever::getLeverMode() {
    if (up_button.pressing()) {
        return LEVER_UP;
    } else if (down_button.pressing()) {
        return LEVER_DOWN;
    }

    return LEVER_OFF;
}

float Lever::getLeverSpeed(Lever::LeverMode leverMode) {
    switch (leverMode) {
        case LEVER_OFF:
            return 0.0;
        case LEVER_UP:
            return 100.0;
        case LEVER_DOWN:
            return -100.0;
        default:
            return 0.0;
    }
}