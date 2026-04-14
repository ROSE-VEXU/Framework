#include "vex.h"

namespace BlackMagic {

DriveErrorProvider::DriveErrorProvider(vex::motor_group& left_motors, vex::motor_group& right_motors):
    left_motors(left_motors),
    right_motors(right_motors) {}

float DriveErrorProvider::getError(float target) {
    float averageDistance = (left_motors.position(vex::rotationUnits::deg) + right_motors.position(vex::rotationUnits::deg)) / 2.0;
    return target - averageDistance;
}

NearestHeadingErrorProvider::NearestHeadingErrorProvider(IHeadingProvider& heading_provider):
    heading_provider(heading_provider) {}

float NearestHeadingErrorProvider::getError(float target) {
    return Utils::getShortestAngleBetween(heading_provider.getHeading(), target);
}

}