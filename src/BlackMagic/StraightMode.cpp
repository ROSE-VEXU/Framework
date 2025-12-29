#include "vex.h"

namespace BlackMagic {

void StraightMode::setTarget(float targetInches) {
    this->target_deg = targetInches * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
    this->speed = 0;
    this->settle_count = 0;
    this->settling_prev_position = 0;
    this->settling_total_position_change = 0;
}

void StraightMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    float curr_distance = utils.getLeftDegrees();
    float curr_error = target_deg - curr_distance;
    speed = linear_pid->getNextValue(curr_error);
}

bool StraightMode::hasSettled(const DriveModeUtilFunctions& utils) {
    float curr_position = utils.getLeftDegrees();
    settling_total_position_change += fabs(curr_position - settling_prev_position);

    if (settling_total_position_change < settling_position_threshold) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_position_change = 0;
    }
    settling_prev_position = curr_position;

    return (settle_count > max_settle_count) ? true : false;
}

DriveSpeeds StraightMode::getSpeeds() {
    return { speed, speed };
}

};
