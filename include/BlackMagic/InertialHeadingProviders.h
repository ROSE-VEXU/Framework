#ifndef INERTIAL_HEADING_PROVIDER_H
#define INERTIAL_HEADING_PROVIDER_H

#include "DriveStates.h"
#include "HeadingProvider.h"

namespace BlackMagic {

class SingleInertialHeadingProvider: public IHeadingProvider {
public:
    SingleInertialHeadingProvider(vex::inertial& imu);
    void calibrate() override;
    Angle getHeading() override;
    void setHeading(float heading) override;
private:
    vex::inertial& imu;
};

class DoubleInertialHeadingProvider: public IHeadingProvider {
public:
    DoubleInertialHeadingProvider(vex::inertial& imu_1, vex::inertial& imu_2);
    void calibrate() override;
    Angle getHeading() override;
    void setHeading(float heading) override;
private:
    vex::inertial& imu_1;
    vex::inertial& imu_2;
};

};

#endif