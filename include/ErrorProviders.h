#ifndef DRIVE_ERROR_PROVIDER_H
#define DRIVE_ERROR_PROVIDER_H

#include "DriveStates.h"
#include "ErrorProvider.h"
#include "HeadingProvider.h"

namespace BlackMagic {

class DriveErrorProvider: public IErrorProvider {
public:
    DriveErrorProvider(const vex::motor_group& left_motors, const vex::motor_group& right_motors);
    float getError(float target) override;
private:
    const vex::motor_group& left_motors;
    const vex::motor_group& right_motors;
};

class NearestHeadingErrorProvider: public IErrorProvider {
public:
    NearestHeadingErrorProvider(const IHeadingProvider& heading_provider);
    float getError(float target) override;
private:
    const IHeadingProvider& heading_provider;
};

};

#endif