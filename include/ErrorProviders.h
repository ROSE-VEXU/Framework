#ifndef DRIVE_ERROR_PROVIDER_H
#define DRIVE_ERROR_PROVIDER_H

#include "DriveStates.h"
#include "ErrorProvider.h"
#include "HeadingProvider.h"

namespace BlackMagic {

class DriveErrorProvider: public IErrorProvider {
public:
    DriveErrorProvider(vex::motor_group& left_motors, vex::motor_group& right_motors, SettleConfig settle_config);
    float getError(float target) override;
    float getRawValue() override;
    bool hasSettled(float target) override;
private:
    vex::motor_group& left_motors;
    vex::motor_group& right_motors;
    float settling_prev_distance;
    float settling_total_distance;
};

class NearestDegreeErrorProvider: public IErrorProvider {
public:
    NearestDegreeErrorProvider(IHeadingProvider& heading_provider, SettleConfig settle_config);
    float getError(float target) override;
    float getRawValue() override;
    bool hasSettled(float target) override;
private:
    IHeadingProvider& heading_provider;
    float settling_prev_heading;
    float settling_total_heading_change;
};

};

#endif