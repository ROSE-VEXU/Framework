#include "vex.h"

namespace BlackMagic {

DriveErrorProvider::DriveErrorProvider(vex::motor_group& left_motors, vex::motor_group& right_motors, SettleConfig settle_config):
    left_motors(left_motors), right_motors(right_motors), settle_config(settle_config) {}

float DriveErrorProvider::getError(float target) {
    return target - getRawValue();
}

bool DriveErrorProvider::hasSettled() {
    return true;
}

NearestDegreeErrorProvider::NearestDegreeErrorProvider(IHeadingProvider& heading_provider, SettleConfig settle_config):
    heading_provider(heading_provider), settle_config(settle_config) {}

float NearestDegreeErrorProvider::getError(float target) {
    Angle target_angle { target, Angle::DEG };
    return Utils::getShortestAngleBetween(heading_provider.getHeading(), target_angle);
}

bool NearestDegreeErrorProvider::hasSettled() {
    return true;
}

}