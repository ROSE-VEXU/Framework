#include "vex.h"
#include <iostream>
#include <string>

namespace BlackMagic {

// Init static member
Robot* Robot::currentReference = nullptr;

Robot::Robot(vex::competition& competitionController): competitionController(competitionController) {
    Robot::currentReference = this;

    competitionController.autonomous(Robot::currentReference->auton);
    competitionController.drivercontrol(Robot::currentReference->driverControl);
}

// Static
void Robot::auton(void) {

}

// Static
void Robot::driverControl(void) {
    // TODO: - Likely needs task shutdown/restart as a cover for auto problems preventing task shutdowns before driver control

    while(true) {
        if (Robot::currentReference == nullptr) continue;

        for (int i=0; i<Robot::currentReference->subsystems.size(); i++) {
            Robot::currentReference->subsystems[i]->opControl();
        }

        vex::wait(VEX_SLEEP_MSEC);
    }
}

Robot& Robot::withAutonomousRoutine(const std::string& name, const std::function<void()>& routine) {
    AutonomousRoutine newRoutine = {
        .name = name,
        .routine = routine
    };
    autoSelector.addRoutine(newRoutine);

    return *this;
}

};
