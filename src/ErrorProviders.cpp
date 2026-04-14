#include "vex.h"

namespace BlackMagic {

DriveErrorProvider::DriveErrorProvider(vex::motor_group& left_motors, vex::motor_group& right_motors):
    left_motors(left_motors),
    right_motors(right_motors) {}

float DriveErrorProvider::getError(float target) {
    return target - getRawValue();
}

float DriveErrorProvider::getRawValue() {
    return (left_motors.position(vex::rotationUnits::deg) + right_motors.position(vex::rotationUnits::deg)) / 2.0;
}

NearestDegreeErrorProvider::NearestDegreeErrorProvider(IHeadingProvider& heading_provider):
    heading_provider(heading_provider) {}

float NearestDegreeErrorProvider::getError(float target) {
    Angle current_angle { getRawValue(), Angle::DEG };
    Angle target_angle { target, Angle::DEG };
    return Utils::getShortestAngleBetween(current_angle, angle);
}

float NearestDegreeErrorProvider::getRawValue() {
    return heading_provider.getHeading().asDegrees();
}

}