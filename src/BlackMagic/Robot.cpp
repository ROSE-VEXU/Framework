#include "vex.h"

namespace BlackMagic {

// Init static member
std::unique_ptr<Robot> Robot::currentReference = nullptr;

Robot::Robot(vex::competition& competitionController): competitionController(competitionController) {
    Robot::currentReference.reset(this);

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
        for (auto& subsystem : Robot::currentReference->subsystems) {
            subsystem->opControl();
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
