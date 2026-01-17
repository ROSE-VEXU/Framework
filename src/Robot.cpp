#include "vex.h"
#include <iostream>
#include <string>

namespace BlackMagic {

// Static field initialization
Robot* Robot::current_robot_reference = nullptr;

Robot::Robot(vex::competition& competition_controller): competition_controller(competition_controller), pre_driver_control([](){}) {
    this->auto_selector = std::make_shared<IAutonomousSelector>();
    this->testing_auto = false;
    Robot::current_robot_reference = this;

    competition_controller.autonomous(Robot::current_robot_reference->auton);
    competition_controller.drivercontrol(Robot::current_robot_reference->driverControl);
}

// Static
void Robot::auton(void) {
    if (Robot::current_robot_reference == nullptr) return;

    AutonomousRoutine selectedAuto = Robot::current_robot_reference->auto_selector->getSelectedRoutine();
    selectedAuto.routine();
}

// Static
void Robot::driverControl(void) {
    Robot::current_robot_reference->pre_driver_control();
    
    while(!(Robot::current_robot_reference->testing_auto)) {
        if (Robot::current_robot_reference == nullptr) continue;

        for (int i=0; i<Robot::current_robot_reference->subsystems.size(); i++) {
            Robot::current_robot_reference->subsystems[i]->opControl();
        }

        vex::wait(VEX_SLEEP_MSEC);
    }
}

Robot& Robot::withAutonomousRoutine(const std::string& name, const std::function<void()>& routine) {
    AutonomousRoutine newRoutine = {
        .name = name,
        .routine = routine
    };
    auto_selector->addRoutine(newRoutine);

    return *this;
}

Robot& Robot::withAutonomousDemoButton(const vex::controller::button button) {
    button.pressed([]() {
        if (Robot::current_robot_reference == nullptr) return;
        if (Robot::current_robot_reference->competition_controller.isCompetitionSwitch()) return; // Prevent accidental demo runs in comp

        Robot::current_robot_reference->testing_auto = true;
        AutonomousRoutine selectedAuto = Robot::current_robot_reference->auto_selector->getSelectedRoutine();
        selectedAuto.routine();
        Robot::current_robot_reference->testing_auto = false;
    });

    return *this;
}

Robot& Robot::withPreDriverControlAction(std::function<void()> pre_driver_control_action) {
    this->pre_driver_control = pre_driver_control_action;
    return *this;
}

};
