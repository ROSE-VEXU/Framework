#include "./BlackMagic/AutonomousPipelineStages.h"
#include "./BlackMagic/PositionProvider.h"

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
    RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, vex::inertial imu_1, vex::inertial imu_2, RotationalOdometryConfig&& config);
    void calibrate();
    void update();
    BlackMagic::Position getPosition();
    float getHeading();
private:
    vex::rotation vert_tracker;
    vex::rotation hori_tracker;
    vex::inertial imu_1;
    vex::inertial imu_2;
    BlackMagic::Position rawPosition;
    RotationalOdometryConfig config;
    RotationalOdometryState prev_state;

    float toRadians(float degrees);
};