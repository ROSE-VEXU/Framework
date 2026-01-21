#include "vex.h"

namespace BlackMagic {

vex::brain& Utils::robot_brain() {
    static vex::brain brain_instance;
    return brain_instance;
}

Angle Utils::getShortestAngleBetween(Angle current_heading, Angle target_heading) {
    Angle angle_error = { target_heading.asRadians() - current_heading.asRadians(), Angle::Unit::RAD };
    return { remainder(angle_error.asDegrees(), 360.0f), Angle::Unit::DEG };
}

Angle Utils::getAngleError(Angle target_heading, Angle current_heading) {
    return getShortestAngleBetween(target_heading, current_heading);
}

int Utils::sign(int value) {
    return value < 0 ? -1 : 1;
}

int Utils::sign(float value) {
    return value < 0.0 ? -1 : 1;
}

float Utils::clamp(float value, float min, float max) {
    return fmax(fmin(value, max), min);
}

BlackMagic::DriveSpeeds Utils::desaturateSpeeds(float linear_speed, float angular_speed) {
    float total_speed = fabs(linear_speed) + fabs(angular_speed);
    float left_speed = linear_speed + angular_speed;
    float right_speed = linear_speed - angular_speed;
    if (total_speed > 100.0f) return { 100.0f * (left_speed/total_speed), 100.0f * (right_speed/total_speed) };
    return { left_speed, right_speed };
}

};