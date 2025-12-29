#include "vex.h"

vex::task drive_task;

void auto1() {
    drive_task = vex::task([]() -> int {
        return robot_drivetrain.driveTask();
    });

    robot_drivetrain.drivePipeline({{0, 12}, 0.0});
}