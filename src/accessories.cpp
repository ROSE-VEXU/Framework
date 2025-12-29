#include "vex.h"

// Pneumatics
vex::pneumatics lever = vex::pneumatics(robot_brain.ThreeWirePort.A);
vex::pneumatics little_will = vex::pneumatics(robot_brain.ThreeWirePort.H);

// Utils
enum Position {
    UP,
    DOWN
};

void setLever(Position position) {
    if (position == Position::UP) {
        little_will.open();
    } else {
        little_will.close();
    }
}

void setLittleWill(Position position) {
    if (position == Position::UP) {
        little_will.close();
    } else {
        little_will.open();
    }
}

Accessories::Accessories(const vex::controller::button& lever_button, const vex::controller::button& little_will_button) : 
    lever_button(lever_button), little_will_button(little_will_button) {
    lever_button.pressed([]() {
        setLever(Position::UP);
    });
    lever_button.released([]() {
        setLever(Position::DOWN);
    });
    little_will_button.pressed([]() {
        setLittleWill(Position::DOWN);
    });
    little_will_button.released([]() {
        setLittleWill(Position::UP);
    });
}

void Accessories::opControl() {}