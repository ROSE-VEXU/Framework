#include "vex.h"

namespace BlackMagic {

DriveErrorProvider::DriveErrorProvider(vex::motor_group& left_motors, vex::motor_group& right_motors, SettleConfig settle_config):
    IErrorProvider(settle_config),
    left_motors(left_motors), right_motors(right_motors) {}

float DriveErrorProvider::getError(float target) {
    return target - getRawValue();
}

float DriveErrorProvider::getRawValue() {
    return (left_motors.position(vex::rotationUnits::deg) + right_motors.position(vex::rotationUnits::deg)) / 2.0;
}

bool DriveErrorProvider::hasSettled(float target) {
    float curr_distance = getRawValue();

    settling_total_distance += fabs(curr_distance - settling_prev_distance);

    if (settling_total_distance < settle_config.reset_settle_threshold) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_distance = 0;
    }
    settling_prev_distance = curr_distance;

    return (settle_count > settle_config.max_settle_count) ? true : false;
}

NearestDegreeErrorProvider::NearestDegreeErrorProvider(IHeadingProvider& heading_provider, SettleConfig settle_config):
    IErrorProvider(settle_config),
    heading_provider(heading_provider) {}

float NearestDegreeErrorProvider::getError(float target) {
    Angle current_angle { getRawValue(), Angle::DEG };
    Angle target_angle { target, Angle::DEG };
    return Utils::getShortestAngleBetween(current_angle, target_angle);
}

float NearestDegreeErrorProvider::getRawValue() {
    return heading_provider.getHeading();
}

bool NearestDegreeErrorProvider::hasSettled(float target) {
    Angle current_angle { getRawValue(), Angle::DEG };
    Angle target_angle { target, Angle::DEG };

    float curr_heading = Utils::getShortestAngleBetween(current_angle, target_angle);
    settling_total_heading_change += fabs(curr_heading - settling_prev_heading);

    if (fabs(settling_total_heading_change) < settle_config.reset_settle_threshold) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_heading_change = 0;
    }
    settling_prev_heading = curr_heading;

    return (settle_count > settle_config.max_settle_count) ? true : false;

}

}