#include "vex.h"

vex::task drive_task;

void skills() {
    // robot_drivetrain.calibrateHeading();

    drive_task = vex::task([]() -> int {
        return robot_drivetrain.driveTask();
    });
    robot_drivetrain.enableDriveTask();

    // robot_drivetrain.driveArc(24.0, 45.0_deg);

    // robot_drivetrain.setPipelinePose({{51.0, 14.0}, 285.0_deg});

    // robot_drivetrain.driveStraight(8.0);
    // robot_drivetrain.drivePipeline({{24.0, 24.0}, 180.0_deg});

    robot_drivetrain.disableDriveTask();
    drive_task.stop();
}