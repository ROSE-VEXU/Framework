#ifndef ROBOT_H
#define ROBOT_H

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "AutonomousSelector.h"
#include "Drivetrain.h"
#include "Subsystem.h"

namespace BlackMagic {

class Robot {
public:
    Robot(vex::competition& competition_controller);

    static Robot* current_robot_reference; // required to run auton & driver control due to vex needing function ptr (can't pass 'this')
    static void auton(void);
    static void driverControl(void);

    template<typename SubsystemType>
    Robot& withSubsystem(SubsystemType&& subsystem) {
        VERIFY_SUBCLASS(std::decay_t<SubsystemType>, Subsystem, "withSubsystem", "subsystem", "Subsystem");
        subsystems.push_back(std::make_shared<std::decay_t<SubsystemType>>(std::forward<SubsystemType>(subsystem)));
        return *this;
    }

    template<typename SubsystemType>
    Robot& withSubsystem(SubsystemType& subsystem) {
        VERIFY_SUBCLASS(std::decay_t<SubsystemType>, Subsystem, "withSubsystem", "subsystem", "Subsystem");
        subsystems.push_back(std::shared_ptr<std::decay_t<SubsystemType>>(&subsystem, [](Subsystem*){}));
        return *this;
    }

    template<typename AutonomousSelectorType>
    Robot& withAutonomousSelector(AutonomousSelectorType& auto_selector) {
        VERIFY_SUBCLASS(std::decay_t<AutonomousSelectorType>, IAutonomousSelector, "withAutonomousSelector", "autoSelector", "IAutonomousSelector");
        this->auto_selector = std::shared_ptr<std::decay_t<AutonomousSelectorType>>(&auto_selector, [](AutonomousSelectorType*){});
        return *this;
    }
    
    Robot& withAutonomousRoutine(const std::string& name, const std::function<void()>& routine);
    Robot& withAutonomousDemoButton(const vex::controller::button button);

    Robot& withPreDriverControlAction(std::function<void()> pre_driver_control_action);
  
private:
    vex::competition& competition_controller;
    std::shared_ptr<IAutonomousSelector> auto_selector;
    std::vector<std::shared_ptr<Subsystem>> subsystems;
    std::function<void()> pre_driver_control;
};

};

#endif