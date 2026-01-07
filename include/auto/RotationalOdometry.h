#ifndef ROTATIONALODOMETRY_H
#define ROTATIONALODOMETRY_H

#include "../BlackMagic/AutonomousPipelineStages.h"
#include "../BlackMagic/HeadingProvider.h"
#include "../BlackMagic/PositionProvider.h"

#define ODOM_WHEEL_DIAM_INCHES 2.06

struct RotationalOdometryConfig {
    float vert_tracker_offset;
    float hori_tracker_offset;
};

struct RotationalOdometryState {
    float vert;
    float hori;
    float heading;
};

class RotationalOdometry: public BlackMagic::IOdometryPipelineStage {
public:
    RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, BlackMagic::IHeadingProvider& heading_provider, RotationalOdometryConfig&& config);
    void calibrate();
    void update();
    // Uses degrees, not inches since straight movement uses deg & PID is tuned for that
    BlackMagic::Position getPosition();
    float getHeading();
private:
    vex::rotation vert_tracker;
    vex::rotation hori_tracker;
    BlackMagic::IHeadingProvider& heading_provider;
    BlackMagic::Position rawPosition;
    RotationalOdometryConfig config;
    RotationalOdometryState prev_state;
};

#endif