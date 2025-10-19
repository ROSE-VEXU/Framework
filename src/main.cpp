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

vex::motor left1 = motor(vex::PORT15, true);
vex::motor left2 = motor(vex::PORT16);
vex::motor left3 = motor(vex::PORT17, true);
vex::motor left4 = motor(vex::PORT18);

vex::motor right1 = motor(vex::PORT10, true);
vex::motor right2 = motor(vex::PORT9);
vex::motor right3 = motor(vex::PORT8, true);
vex::motor right4 = motor(vex::PORT7);

vex::rotation vert_tracking = rotation(PORT20, false);
vex::rotation hori_tracking = rotation(PORT19, true);
vex::inertial imu_1 = inertial(PORT6);
vex::inertial imu_2 = inertial(PORT5);


const float WHEEL_DIAM_INCHES = 2.75;

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  static BlackMagic::Robot robot(competitionController);
  robot
    .withSubsystem(
      BlackMagic::Drivetrain(
        vex::motor_group(left1, left2, left3, left4),
        vex::motor_group(right1, right2, right3, right4),
        WHEEL_DIAM_INCHES,
        BlackMagic::PID(0.0, 0.0, 0.0)
      )
        .withControllerMovement(BlackMagic::TankDriveControl(mainController))
        .withAutonomousPipeline(
          BlackMagic::AutonomousPipeline()
            .withOdometrySource(RotationalOdometry(vert_tracking, hori_tracking, imu_1, imu_2))
        )
    );

  // Don't leave scope to avoid destroying the robot object
  while (1) {
    vex::wait(100, msec);
  }
}
