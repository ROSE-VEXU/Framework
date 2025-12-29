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

vex::brain robot_brain = brain();
vex::competition comp_controller;
vex::controller main_controller;

vex::motor left1 = motor(vex::PORT17);
vex::motor left2 = motor(vex::PORT16, true);
vex::motor left3 = motor(vex::PORT18);
vex::motor left4 = motor(vex::PORT19, true);
vex::motor_group left_motors = motor_group(left1, left2, left3, left4);

vex::motor right1 = motor(vex::PORT13, true);
vex::motor right2 = motor(vex::PORT15);
vex::motor right3 = motor(vex::PORT14, true);
vex::motor right4 = motor(vex::PORT12);
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
        .withControllerMovement(ArcadeDriveControl(main_controller.Axis3, main_controller.Axis1))
        .withAutonomousPipeline(
          BlackMagic::AutonomousPipeline()
            .withOdometrySource(RotationalOdometry(vert_tracking, hori_tracking, imu_1, imu_2, { .vert_tracker_offset=0.0, .hori_tracker_offset=-2.0 }))
            .withSpeedController(DriveToPoseSpeedController())
        )
    )
    .withSubsystem(
      // In button, Out button
      Intake(main_controller.ButtonR2, main_controller.ButtonR1)
    )
    .withSubsystem(
      // Up Button, Down Button
      Lever(main_controller.ButtonL1, main_controller.ButtonL2)
    )
    .withAutonomousRoutine("Auto 1", auto1)
    .withAutonomousDemoButton(main_controller.ButtonUp)
    .withAutonomousSelector(LimitSwitchAutoSelector())
    .withPreDriverControlAction([]() {
      // End any tasks or items that may interfere with driver control
      robot_drivetrain.disableDriveTask(); // If auto gets cut short and pid task runs during driver, controller input will be overridden
    });

  // Don't leave scope to avoid destroying the robot object
  while (1) {
    vex::wait(100, msec);
  }
}
