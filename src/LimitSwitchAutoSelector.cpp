#include "vex.h"

vex::limit backSwitch = vex::limit(BlackMagic::Utils::robot_brain().ThreeWirePort.B);
vex::limit frontSwitch = vex::limit(BlackMagic::Utils::robot_brain().ThreeWirePort.C);

// Static field initialization
LimitSwitchAutoSelector* LimitSwitchAutoSelector::current_limit_selector_reference = nullptr;

LimitSwitchAutoSelector::LimitSwitchAutoSelector() : selectedAuto(0) {
    LimitSwitchAutoSelector::current_limit_selector_reference = this;
    backSwitch.pressed(LimitSwitchAutoSelector::prevAuto);
    frontSwitch.pressed(LimitSwitchAutoSelector::nextAuto);
    displaySelectedAuto();
}

BlackMagic::AutonomousRoutine LimitSwitchAutoSelector::getSelectedRoutine() {
    if (routines.empty()) {
        return BlackMagic::AutonomousRoutine{"No Routines Available", [](){}};
    }

    return routines[selectedAuto];
}

void LimitSwitchAutoSelector::addRoutine(BlackMagic::AutonomousRoutine routine) {
    routines.push_back(routine);
    displaySelectedAuto();
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
    BlackMagic::Utils::robot_brain().Screen.clearScreen();
    BlackMagic::Utils::robot_brain().Screen.setPenColor(vex::color(212, 212, 212));
    BlackMagic::Utils::robot_brain().Screen.setFillColor(vex::color::black);
    BlackMagic::Utils::robot_brain().Screen.printAt(12, 218, "HW v2.0.0");
    BlackMagic::Utils::robot_brain().Screen.printAt(196, 218, "ROSE VEXU");
    BlackMagic::Utils::robot_brain().Screen.printAt(376, 218, "SW v0.3.5");

    BlackMagic::Utils::robot_brain().Screen.setPenColor(vex::color::black);
    BlackMagic::Utils::robot_brain().Screen.drawRectangle(0, 40, 480, 40, vex::color(128, 0, 0));
    BlackMagic::Utils::robot_brain().Screen.setPenColor(vex::color(212, 212, 212));
    BlackMagic::Utils::robot_brain().Screen.setFillColor(vex::color(128, 0, 0));
    BlackMagic::Utils::robot_brain().Screen.printAt(12, 64, getSelectedRoutine().name.c_str());
}   