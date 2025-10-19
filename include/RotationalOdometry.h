#include "./BlackMagic/PipelineStages.h"

class RotationalOdometry: public BlackMagic::IOdometryPipelineStage {
public:
    RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, vex::inertial imu_1, vex::inertial imu_2);
    void calibrate();
    void update();
    float getX();
    float getY();
private:
    vex::rotation vert_tracker;
    vex::rotation hori_tracker;
    vex::inertial imu_1;
    vex::inertial imu_2;
    float xPosition;
    float yPosition;
};