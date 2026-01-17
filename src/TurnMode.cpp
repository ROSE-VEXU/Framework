#include "vex.h"

namespace BlackMagic {

void TurnMode::setTarget(Angle target_heading) {
    this->target_heading = target_heading;
    this->left_speed = 0;
    this->right_speed = 0;
    this->settle_count = 0;
    this->settling_prev_heading = 0;
    this->settling_total_heading_change = 0;
}

void TurnMode::run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) {
    float curr_error = Utils::getShortestAngleBetween(drive_state.heading, this->target_heading);
    float turn_speed = angular_pid->getNextValue(curr_error);
    float prev_turn_speed = left_speed;

    left_speed = Utils::sign(turn_speed) * fabs(turn_speed);
    right_speed = -1 * Utils::sign(turn_speed) * fabs(turn_speed);

    if (fabs(turn_speed) < fabs(prev_turn_speed)) decelerating = true;
}

bool TurnMode::hasSettled(const DrivetrainState& drive_state) {
    if (!decelerating) return false;

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

DriveSpeeds TurnMode::getSpeeds() {
    return { left_speed, right_speed };
}

};
