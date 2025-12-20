#include "vex.h"

vex::motor leftLever = vex::motor(vex::PORT5);
vex::motor rightLever = vex::motor(vex::PORT3, true);

Lever::Lever(const vex::controller& mainController) : 
    mainController(mainController) {
}

void Lever::opControl() {
    float speed = getLeverSpeed(getLeverMode());
    leftLever.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    rightLever.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

Lever::LeverMode Lever::getLeverMode() {
    bool topPressed = mainController.ButtonL1.pressing();
    bool bottomPressed = mainController.ButtonL2.pressing();

    if (topPressed) {
        return LEVER_UP;
    } else if (bottomPressed) {
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