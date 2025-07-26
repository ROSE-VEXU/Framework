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

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  BlackMagic::Robot& robot =
  BlackMagic::Robot{competitionController}
    .withSubsystem(
      BlackMagic::Drivetrain{}
        .withPID(BlackMagic::PID{0.0, 0.0, 0.0})
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
