#include "vex.h"

namespace BlackMagic {

RadialArcMode::RadialArcMode() {}

void RadialArcMode::setTarget(float radius_inches, Angle target_heading) {
    this->radius_deg = radius_inches * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
    this->target_heading = target_heading;
    this->linear_speed = 0;
    this->angular_speed = 0;
    this->settle_count = 0;
    this->settling_prev_heading = 0;
    this->settling_total_heading_change = 0;

    setArcTarget(radius_inches, target_heading);
}

void RadialArcMode::run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) {
    linear_speed = linear_pid.getNextValue(target_arc_length_deg);
    angular_speed = angular_pid.getNextValue(getCurrentTargetAngle(radius_deg, curr_distance));
}

bool RadialArcMode::hasSettled(const DrivetrainState& drive_state) {
    float curr_heading = Utils::getShortestAngleBetween(drive_state.heading, this->target_heading);
    settling_total_heading_change += fabs(curr_heading - settling_prev_heading);

    if (fabs(settling_total_heading_change) < TURN_DRIVE_SETTLE_HEADING_THRESHOLD) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_heading_change = 0;
    }
    settling_prev_heading = curr_heading;

    return (settle_count > TURN_DRIVE_SETTLE_COUNT) ? true : false;
}

DriveSpeeds RadialArcMode::getSpeeds() {
    return BlackMagic::Utils::desaturateSpeeds(linear_speed, angular_speed);
}

void RadialArcMode::setArcTarget(float target_radius_inches, Angle target_angle) {
    float arc_length_inches = 2 * M_PI * target_radius_inches * (target_angle/360.0f);

    target_arc_length_deg = arc_length_inches * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
}

Angle RadialArcMode::getCurrentTargetAngle(float arc_radius, float driven_arc_length) {
    return { (driven_arc_length * 360.0) / (2 * M_PI * arc_radius), BlackMagic::Angle::Unit::DEG };
}


};
