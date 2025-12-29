#include "vex.h"

namespace BlackMagic {

void TurnMode::setTarget(float targetHeading) {
    this->target_heading = targetHeading;
    this->left_speed = 0;
    this->right_speed = 0;
    this->settling_prev_heading = 0;
    this->settling_total_heading_change = 0;
}

void TurnMode::run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    float curr_heading = utils.getHeading();
    float curr_error = Utils::getShortestAngleBetween(curr_heading, this->target_heading);
    float turn_speed = angular_pid->getNextValue(curr_error);

    this->left_speed = turn_speed;
    this->right_speed = -turn_speed;
}

bool TurnMode::hasSettled(const DriveModeUtilFunctions& utils) {
    float curr_heading = Utils::getShortestAngleBetween(utils.getHeading(), this->target_heading);
    settling_total_heading_change += fabs(curr_heading - settling_prev_heading);

    if (fabs(settling_total_heading_change) < settling_angle_threshold) {
        settle_count++;
    } else {
        settle_count = 0;
        settling_total_heading_change = 0;
    }
    settling_prev_heading = curr_heading;

    return (settle_count > max_settle_count) ? true : false;
}

DriveSpeeds TurnMode::getSpeeds() {
    return { left_speed, right_speed };
}

};
