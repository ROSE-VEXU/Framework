#include "vex.h"

vex::task drive_task;

void auto1() {
    drive_task = vex::task([]() -> int {
        return robot_drivetrain.driveTask();
    });
    robot_drivetrain.enableDriveTask();

    // robot_drivetrain.driveStraight(48.0);
    // robot_drivetrain.driveTurn(90.0_deg);
    robot_drivetrain.drivePipeline({{0.0, 24.0}, 90.0_deg});

    robot_drivetrain.disableDriveTask();
    drive_task.stop();
}