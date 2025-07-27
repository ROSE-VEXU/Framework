/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       austin                                                    */
/*    Created:      7/17/2025, 6:59:45 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// define your global instances of motors and other devices here

void setup() {

}

void demofunc() {}

vex::competition competitionController;
vex::controller mainController;

vex::motor left1 = motor(vex::PORT1);
vex::motor left2 = motor(vex::PORT2);
vex::motor left3 = motor(vex::PORT3);
vex::motor left4 = motor(vex::PORT4);
vex::motor left5 = motor(vex::PORT5);

vex::motor right1 = motor(vex::PORT6);
vex::motor right2 = motor(vex::PORT7);
vex::motor right3 = motor(vex::PORT8);
vex::motor right4 = motor(vex::PORT9);
vex::motor right5 = motor(vex::PORT10);

const float WHEEL_DIAM_INCHES = 3.25;

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  BlackMagic::Robot& robot =
  BlackMagic::Robot{competitionController}
    .withSubsystem(
      BlackMagic::Drivetrain(
        vex::motor_group(left1, left2, left3, left4, left5),
        vex::motor_group(right1, right2, right3, right4, right5),
        WHEEL_DIAM_INCHES,
        BlackMagic::PID(0.0, 0.0, 0.0)
      )
        .withControllerMovement(BlackMagic::TankDriveControl(mainController))
        .withAlignmentCorrection(0.4)
        .withAutonomousPipeline(
          BlackMagic::AutonomousPipeline{}
            .withOdometrySource(BlackMagic::OdometryPipelineStage{})
            .withLocalizationSource(BlackMagic::LocalizationPipelineStage{})
            .withSpeedController(BlackMagic::SpeedController{})
        )
    )
    .withAutonomousRoutine("Routine 1", demofunc);
}
