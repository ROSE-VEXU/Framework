#include "vex.h"
#include <iostream>
#include <string>

namespace BlackMagic {

// Static field initialization
Robot* Robot::current_robot_reference = nullptr;

Robot::Robot(vex::competition& competitionController): competitionController(competitionController) {
    Robot::current_robot_reference = this;

    competitionController.autonomous(Robot::current_robot_reference->auton);
    competitionController.drivercontrol(Robot::current_robot_reference->driverControl);
}

// Static
void Robot::auton(void) {
    if (Robot::current_robot_reference == nullptr) return;

    AutonomousRoutine selectedAuto = Robot::current_robot_reference->autoSelector.getSelectedRoutine();
    selectedAuto.routine();
}

// Static
void Robot::driverControl(void) {
    // TODO: - Likely needs task shutdown/restart as a cover for auto problems preventing task shutdowns before driver control

    while(true) {
        if (Robot::current_robot_reference == nullptr) continue;

        for (int i=0; i<Robot::current_robot_reference->subsystems.size(); i++) {
            Robot::current_robot_reference->subsystems[i]->opControl();
        }

        vex::wait(VEX_SLEEP_MSEC);
    }
}

Robot& Robot::withAutonomousSelector(IAutonomousSelector&& autoSelector) {
    this->autoSelector = std::move(autoSelector);
    return *this;
}

Robot& Robot::withAutonomousRoutine(const std::string& name, const std::function<void()>& routine) {
    AutonomousRoutine newRoutine = {
        .name = name,
        .routine = routine
    };
    autoSelector.addRoutine(newRoutine);

    return *this;
}

Robot& Robot::withAutonomousDemoButton(const vex::controller::button button) {
    button.pressed([]() {
        if (Robot::current_robot_reference == nullptr) return;

        AutonomousRoutine selectedAuto = Robot::current_robot_reference->autoSelector.getSelectedRoutine();
        selectedAuto.routine();
    });

    return *this;
}

};
