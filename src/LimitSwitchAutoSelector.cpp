#include "vex.h"

vex::limit backSwitch = vex::limit(robot_brain.ThreeWirePort.B);
vex::limit frontSwitch = vex::limit(robot_brain.ThreeWirePort.C);

// Static field initialization
LimitSwitchAutoSelector* LimitSwitchAutoSelector::current_limit_selector_reference = nullptr;

LimitSwitchAutoSelector::LimitSwitchAutoSelector() : selectedAuto(0) {
    LimitSwitchAutoSelector::current_limit_selector_reference = this;
    backSwitch.pressed(LimitSwitchAutoSelector::prevAuto);
    frontSwitch.pressed(LimitSwitchAutoSelector::nextAuto);
}

BlackMagic::AutonomousRoutine LimitSwitchAutoSelector::getSelectedRoutine() {
    if (routines.empty()) {
        return BlackMagic::AutonomousRoutine{"No Routines Available", [](){}};
    }

    return routines[selectedAuto];
}

void LimitSwitchAutoSelector::updateSelectedAuto(int direction) {
    selectedAuto += direction;
    if (selectedAuto < 0) {
        selectedAuto += routines.size();
    }
    selectedAuto %= routines.size();

    displaySelectedAuto();
}

void LimitSwitchAutoSelector::displaySelectedAuto() {
    robot_brain.Screen.clearScreen();
    robot_brain.Screen.printAt(32, 24, getSelectedRoutine().name.c_str());
}