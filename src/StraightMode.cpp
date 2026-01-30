#include "vex.h"

namespace BlackMagic {

StraightMode::StraightMode() {}

void StraightMode::setTarget(float target_inches, Angle target_heading) {
    this->target_deg = target_inches * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
    this->target_heading = target_heading;
    this->decelerating = false;
    this->linear_speed = 0;
    this->angular_speed = 0;
    this->settle_count = 0;
    this->settling_prev_left = 0;
    this->settling_prev_right = 0;
    this->settling_total_left = 0;
    this->settling_total_right = 0;
}

void StraightMode::run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) {
    float curr_distance = (drive_state.left_degrees + drive_state.right_degrees) / 2.0;
    float curr_distance_error = target_deg - curr_distance;
    float curr_heading_error = Utils::getShortestAngleBetween(drive_state.heading, target_heading);
    float prev_linear_speed = linear_speed;

    linear_speed = linear_pid.getNextValue(curr_distance_error);
    angular_speed = angular_pid.getNextValue(curr_heading_error);

    // Scale angular to avoid curves at the start
    angular_speed *= (fabs(linear_speed) / linear_pid.getMaxSpeed());

    if (fabs(linear_speed) < fabs(prev_linear_speed)) decelerating = true;
}

bool StraightMode::hasSettled(const DrivetrainState& drive_state) {
    if (!decelerating) return false;

    float curr_left = drive_state.left_degrees;
    float curr_right = drive_state.right_degrees;

    settling_total_left += fabs(curr_left - settling_prev_left);
    settling_total_right += fabs(curr_right - settling_prev_right);

    if (settling_total_left < STRAIGHT_DRIVE_SETTLE_DEG_THRESHOLD &&
        settling_total_right < STRAIGHT_DRIVE_SETTLE_DEG_THRESHOLD) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_left = 0;
        settling_total_right = 0;
    }
    settling_prev_left = curr_left;
    settling_prev_right = curr_right;

    return (settle_count > STRAIGHT_DRIVE_SETTLE_COUNT) ? true : false;
}

DriveSpeeds StraightMode::getSpeeds() {
    return BlackMagic::Utils::desaturateSpeeds(linear_speed, angular_speed);
}

};
