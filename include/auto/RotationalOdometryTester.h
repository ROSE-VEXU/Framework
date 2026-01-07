#ifndef ROTATIONALODOMETRYTESTER_H
#define ROTATIONALODOMETRYTESTER_H

#include "../BlackMagic/Subsystem.h"
#include "./RotationalOdometry.h"

class RotationalOdometryTester: public BlackMagic::Subsystem {
public:
    RotationalOdometryTester(vex::rotation vert_tracker, vex::rotation hori_tracker, BlackMagic::IHeadingProvider& heading_provider, RotationalOdometryConfig&& config);
    void opControl() override;
private:
    RotationalOdometry odometry;
};

#endif