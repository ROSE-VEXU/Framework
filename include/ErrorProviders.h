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
private:
    vex::motor_group& left_motors;
    vex::motor_group& right_motors;
};

class NearestDegreeErrorProvider: public IErrorProvider {
public:
    NearestDegreeErrorProvider(IHeadingProvider& heading_provider, SettleConfig settle_config);
    float getError(float target) override;
    float getRawValue() override;
private:
    IHeadingProvider& heading_provider;
};

};

#endif