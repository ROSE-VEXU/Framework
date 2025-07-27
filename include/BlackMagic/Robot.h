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

    template<typename SubsystemType>
    Robot& withSubsystem(SubsystemType&& subsystem) {
        VERIFY_SUBCLASS(SubsystemType, Subsystem, "withSubsystem", "subsystem", "Subsystem");
        subsystems.push_back(std::make_unique<std::decay_t<SubsystemType>>(std::forward<SubsystemType>(subsystem)));
        return *this;
    }

    Robot& withAutonomousRoutine(const std::string& name, std::function<void()> routine);
private:
    vex::competition& competitionController;
    AutonomousSelector autoSelector;
    std::vector<std::unique_ptr<Subsystem>> subsystems;
};

};

#endif