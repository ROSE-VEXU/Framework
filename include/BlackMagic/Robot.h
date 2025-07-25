#ifndef ROBOT_H
#define ROBOT_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "AutonomousSelector.h"
#include "Subsystem.h"

namespace BlackMagic {

class Robot {
public:
    Robot(vex::competition& competitionController);
    static std::unique_ptr<Robot> currentReference; // required to run auton & driver control due to vex needing function ptr (can't pass 'this')
    static void auton(void);
    static void driverControl(void);

    template<typename SubsystemT>
    Robot& withSubsystem(SubsystemT&& subsystem) {
        static_assert(std::is_base_of<Subsystem, std::decay_t<SubsystemT>>::value, "Robot.withSubsystem requires a Subsystem type object to work.");
        subsystems.push_back(std::make_unique<std::decay_t<SubsystemT>>(std::forward<SubsystemT>(subsystem)));
        return *this;
    }

    // Robot& withAutonomousRoutine(const std::string& name, void(*)(void) routine);
    // Robot& withAutonomousRoutine(const std::string& name, std::function<void()>* routine);
    Robot& withAutonomousRoutine(const std::string& name, const std::function<void()>& routine);
private:
    vex::competition& competitionController;
    AutonomousSelector autoSelector;
    std::vector<std::unique_ptr<Subsystem>> subsystems;
};

};

#endif