#ifndef DRIVE_ERROR_PROVIDER_H
#define DRIVE_ERROR_PROVIDER_H

#include "DriveStates.h"
#include "ErrorProvider.h"
#include "HeadingProvider.h"

namespace BlackMagic {

class DriveErrorProvider: public IErrorProvider<float> {
    DriveErrorProvider(const vex::motor_group& left_motors, const vex::motor_group& right_motors);
private:
    const vex::motor_group& left_motors;
    const vex::motor_group& right_motors;
};

class NearestHeadingErrorProvider: public IErrorProvider<Angle> {
    NearestHeadingErrorProvider(const IHeadingProvider& heading_provider);
private:
    const IHeadingProvider& heading_provider;
};

};

#endif