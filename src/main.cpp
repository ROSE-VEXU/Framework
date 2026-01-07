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
vex::competition comp_controller;
vex::controller main_controller;

vex::motor left1 = motor(vex::PORT19, vex::ratio6_1);
vex::motor left2 = motor(vex::PORT18, vex::ratio6_1, true);
vex::motor left3 = motor(vex::PORT17, vex::ratio6_1);
vex::motor left4 = motor(vex::PORT16, vex::ratio6_1, true);
vex::motor_group left_motors = motor_group(left1, left2, left3, left4);

vex::motor right1 = motor(vex::PORT15, vex::ratio6_1, true);
vex::motor right2 = motor(vex::PORT14, vex::ratio6_1);
vex::motor right3 = motor(vex::PORT13, vex::ratio6_1, true);
vex::motor right4 = motor(vex::PORT12, vex::ratio6_1);
vex::motor_group right_motors = motor_group(right1, right2, right3, right4);

vex::rotation vert_tracking = rotation(PORT5, true);
vex::rotation hori_tracking = rotation(PORT2, true);
vex::inertial imu_1 = inertial(PORT3);
vex::inertial imu_2 = inertial(PORT4);

BlackMagic::DoubleInertialHeadingProvider double_imu_heading_provider(imu_1, imu_2);

BlackMagic::Drivetrain robot_drivetrain = BlackMagic::Drivetrain(left_motors, right_motors, double_imu_heading_provider);
BlackMagic::PIDConfig linear_pid_config = { PID_SETTING_DISABLE , 80.0, 5.0, PID_SETTING_DISABLE};
BlackMagic::PIDConfig angular_pid_config = { PID_SETTING_DISABLE , 80.0, PID_SETTING_DISABLE, PID_SETTING_DISABLE};

void setup() {
  imu_1.calibrate(3);
  imu_2.calibrate(3);
}

int main() {
  setup(); // pre_auton replacement since robot handles game lifecycle

  static BlackMagic::Robot robot(comp_controller);

  robot
    .withSubsystem(
      robot_drivetrain
        .withLinearPID(BlackMagic::PID(0.15, {0.0015, 2160.0, 720.0}, 1, linear_pid_config))
        .withAngularPID(BlackMagic::PID(1.45, {0.0, 0.0, 0.0}, 10.0, angular_pid_config))
        .withControllerMovement(BlackMagic::TankDriveControl(main_controller.Axis3, main_controller.Axis2))
        .withAutonomousPipeline(
          BlackMagic::AutonomousPipeline()
            .withOdometrySource(RotationalOdometry(vert_tracking, hori_tracking, double_imu_heading_provider, { .vert_tracker_offset=0.241, .hori_tracker_offset=-2.111 }))
            .withSpeedController(SimpleDriveToPoint())
        )
    )
    .withSubsystem(
      // In button, Out button
      Intake(main_controller.ButtonR1, main_controller.ButtonR2)
    )
    .withSubsystem(
      // Up Button, Down Button
      Lever(main_controller.ButtonL1, main_controller.ButtonL2)
    )
    .withSubsystem(
      // Lever Button, Little Will Button
      Accessories(main_controller.ButtonRight, main_controller.ButtonY)
    )
    .withSubsystem(
      RotationalOdometryTester(vert_tracking, hori_tracking, double_imu_heading_provider, { .vert_tracker_offset=-0.241, .hori_tracker_offset=2.111 })//{ .vert_tracker_offset=-0.241, .hori_tracker_offset=-2.111 })
    )
    .withAutonomousSelector(LimitSwitchAutoSelector())
    .withAutonomousRoutine("Skills left", skills)
    .withAutonomousDemoButton(main_controller.ButtonUp)
    .withPreDriverControlAction([]() {
      // End any tasks or items that may interfere with driver control
      robot_drivetrain.disableDriveTask(); // If auto gets cut short and pid task runs during driver, controller input will be overridden
    });

  // Don't leave scope to avoid destroying the robot object
  while (1) {
    vex::wait(100, msec);
  }
}
