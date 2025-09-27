/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       austin                                                    */
/*    Created:      7/17/2025, 6:59:45 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>

using namespace vex;

// define your global instances of motors and other devices here

void setup() {
  std::cout << "setup!";
}

void demofunc() {}

vex::competition competitionController;
vex::controller mainController;

vex::motor left1 = motor(vex::PORT1);
vex::motor left2 = motor(vex::PORT2);
vex::motor left3 = motor(vex::PORT3);

vex::motor right1 = motor(vex::PORT8);
vex::motor right2 = motor(vex::PORT9);
vex::motor right3 = motor(vex::PORT10);

const float WHEEL_DIAM_INCHES = 2.75;

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  static BlackMagic::Robot robot(competitionController);
  robot
    .withSubsystem(
      BlackMagic::Drivetrain(
        vex::motor_group(left1, left2, left3),
        vex::motor_group(right1, right2, right3),
        WHEEL_DIAM_INCHES,
        BlackMagic::PID(0.0, 0.0, 0.0)
      )
        .withControllerMovement(BlackMagic::TankDriveControl(mainController))
    );
//         // .withAlignmentCorrection(0.4)
//         // .withAutonomousPipeline(
//         //   BlackMagic::AutonomousPipeline{}
//         //     .withOdometrySource(BlackMagic::OdometryPipelineStage{})
//         //     .withLocalizationSource(BlackMagic::LocalizationPipelineStage{})
//         //     .withSpeedController(BlackMagic::SpeedController{})
//         // )
//     // )
//     // .withAutonomousRoutine("Routine 1", demofunc)
//     // .withAutonomousRoutine("Routine 2", []() {

//     // });

  // Don't leave scope to avoid destroying the robot object
  while (1) {
    vex::wait(100, msec);
  }
}

// int main () {
//   BlackMagic::Drivetrain dt(
//     vex::motor_group(left1, left2, left3),
//     vex::motor_group(right1, right2, right3),
//     WHEEL_DIAM_INCHES,
//     BlackMagic::PID(0.0, 0.0, 0.0)
//   );


//   while (1) {
//     vex::wait(100, msec);
//   }
// }
