#include "vex.h"

namespace BlackMagic {

SingleInertialHeadingProvider::SingleInertialHeadingProvider(vex::inertial& imu): imu(imu) {}

void SingleInertialHeadingProvider::calibrate() {
    imu.calibrate(3);

    while (imu.isCalibrating() || imu.isCalibrating()) {
        vex::wait(VEX_SLEEP_MSEC);
    }
}

Angle SingleInertialHeadingProvider::getHeading() {
    return { imu.heading(vex::degrees), Angle::Unit::DEG };
}

void SingleInertialHeadingProvider::setHeading(float heading) {
    imu.setHeading(heading, vex::rotationUnits::deg);
}


DoubleInertialHeadingProvider::DoubleInertialHeadingProvider(vex::inertial& imu_1, vex::inertial& imu_2): imu_1(imu_1), imu_2(imu_2) {}

void DoubleInertialHeadingProvider::calibrate() {
    imu_1.calibrate(3);
    imu_2.calibrate(3);

    while (imu_1.isCalibrating() || imu_2.isCalibrating()) {
        vex::wait(VEX_SLEEP_MSEC);
    } 
}

Angle DoubleInertialHeadingProvider::getHeading() {
    float imu_1_heading = imu_1.heading(vex::rotationUnits::deg);
    float imu_2_heading = imu_2.heading(vex::rotationUnits::deg);
    float max = std::max(imu_1_heading, imu_2_heading);
    float min = std::min(imu_1_heading, imu_2_heading);

    if (max - min > 180) { // Massive difference between the two means that they're on opposite sides of 360
        max -= 180;
        min += 180;
        return { fmod(((max+min) / 2.0) + 180.0, 360.0), Angle::Unit::DEG }; // Mod is necessary to prevent returning > 360
    }

    return { ((max+min) / 2.0), Angle::Unit::DEG };
}

void DoubleInertialHeadingProvider::setHeading(float heading) {
    imu_1.setHeading(heading, vex::rotationUnits::deg);
    imu_2.setHeading(heading, vex::rotationUnits::deg);
}

};