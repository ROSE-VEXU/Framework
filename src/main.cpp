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

// extern vex::brain robot_brain;
vex::competition comp_controller;
vex::controller main_controller;

vex::motor left1 = motor(vex::PORT17, true);
vex::motor left2 = motor(vex::PORT16);
vex::motor left3 = motor(vex::PORT18, true);
vex::motor left4 = motor(vex::PORT19);
vex::motor_group left_motors = motor_group(left1, left2, left3, left4);

vex::motor right1 = motor(vex::PORT13);
vex::motor right2 = motor(vex::PORT15, true);
vex::motor right3 = motor(vex::PORT14);
vex::motor right4 = motor(vex::PORT12, true);
vex::motor_group right_motors = motor_group(right1, right2, right3, right4);

vex::rotation vert_tracking = rotation(PORT20, false);
vex::rotation hori_tracking = rotation(PORT19, true);
vex::inertial imu_1 = inertial(PORT6);
vex::inertial imu_2 = inertial(PORT5);

BlackMagic::Drivetrain robot_drivetrain = BlackMagic::Drivetrain(left_motors, right_motors, imu_1);

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  static BlackMagic::Robot robot(comp_controller);

  robot
    .withSubsystem(
      robot_drivetrain
        .withLinearPID(BlackMagic::PID(0.0, 0.0, 0.0, 5.0, 5.0))
        .withAngularPID(BlackMagic::PID(0.0, 0.0, 0.0, 5.0, 5.0))
        .withControllerMovement(ArcadeDriveControl(main_controller))//BlackMagic::TankDriveControl(main_controller))
        .withAutonomousPipeline(
          BlackMagic::AutonomousPipeline()
            .withOdometrySource(RotationalOdometry(vert_tracking, hori_tracking, imu_1, imu_2, { .vert_tracker_offset=0.0, .hori_tracker_offset=-2.0 }))
            .withSpeedController(DriveToPoseSpeedController())
        )
    )
    .withSubsystem(
      Intake(main_controller)
    )
    .withSubsystem(
      Lever(main_controller)
    )
    .withAutonomousRoutine("Auto 1", auto1)
    .withAutonomousDemoButton(main_controller.ButtonUp);

  // Don't leave scope to avoid destroying the robot object
  while (1) {
    vex::wait(100, msec);
  }
}
