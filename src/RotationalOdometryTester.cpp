#include "vex.h"
#include <algorithm>

RotationalOdometryTester::RotationalOdometryTester(vex::rotation vert_tracker, vex::rotation hori_tracker, BlackMagic::IHeadingProvider& heading_provider, RotationalOdometryConfig&& config):
    odometry(vert_tracker, hori_tracker, heading_provider, std::move(config)) {}

void RotationalOdometryTester::opControl() {
    odometry.update();
    BlackMagic::Position pos = odometry.getPosition();
    BlackMagic::Utils::robot_brain().Screen.clearScreen();
    BlackMagic::Utils::robot_brain().Screen.printAt(10, 10, "X: %.2f, Y: %.2f", pos.x, pos.y);
};
