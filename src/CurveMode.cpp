#include "vex.h"

namespace BlackMagic {

CurveMode::CurveMode() {}

void CurveMode::setTarget(float target_inches, std::vector<CurveKeyframe> keyframes) {
    this->target_deg = target_inches * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
    this->keyframes = keyframes;
    this->curr_keyframe_index = 0;
    this->linear_speed = 0;
    this->angular_speed = 0;
    this->settle_count = 0;
    this->settling_prev_left = 0;
    this->settling_prev_right = 0;
    this->settling_total_left = 0;
    this->settling_total_right = 0;
}

void CurveMode::run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) {
    float curr_distance = drive_state.left_degrees;
    float curr_distance_error = target_deg - curr_distance;

    // Advance to next keyframe if we've driven to that %
    if (curr_keyframe_index < keyframes.size()-1 &&
        curr_distance_error >= target_deg * keyframes[curr_keyframe_index+1].pct) {
            curr_keyframe_index++;
    }

    float curr_segment_error = curr_distance_error - (target_deg * keyframes[curr_keyframe_index].pct);
    float segment_pct_length = (curr_keyframe_index == keyframes.size()-1) ? 1.0-keyframes[curr_keyframe_index].pct 
                                                                           : keyframes[curr_keyframe_index+1].pct - keyframes[curr_keyframe_index].pct;
    float curr_segment_pct_driven = curr_segment_error / (target_deg * segment_pct_length);
    float curr_segment_smoothing_pct = curr_segment_pct_driven / keyframes[curr_keyframe_index].smooth_pct;

    float curr_heading_error = curr_segment_smoothing_pct * Utils::getShortestAngleBetween(drive_state.heading, keyframes[curr_keyframe_index].heading);
    float prev_linear_speed = linear_speed;

    linear_speed = linear_pid.getNextValue(curr_distance_error);
    angular_speed = angular_pid.getNextValue(curr_heading_error);
}

bool CurveMode::hasSettled(const DrivetrainState& drive_state) {
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

DriveSpeeds CurveMode::getSpeeds() {
    return BlackMagic::Utils::desaturateSpeeds(linear_speed, angular_speed);
}

};
