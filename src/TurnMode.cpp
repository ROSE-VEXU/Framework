#include "vex.h"

namespace BlackMagic {

TurnMode::TurnMode() {}

void TurnMode::setTarget(Angle target_heading) {
    this->target_heading = target_heading;
    this->left_speed = 0;
    this->right_speed = 0;
    this->settle_count = 0;
    this->settling_prev_heading = 0;
    this->settling_total_heading_change = 0;
}

void TurnMode::setErrorProviders(IErrorProvider& error_provider) {
    this->error_provider = &error_provider;
}

void TurnMode::run(PID& linear_pid, PID& angular_pid) {
    float curr_angular_error = error_provider->getError(target_heading);
    float turn_speed = angular_pid.getNextValue(curr_angular_error);
    float prev_turn_speed = left_speed;

    left_speed = Utils::sign(turn_speed) * fabs(turn_speed);
    right_speed = -1 * Utils::sign(turn_speed) * fabs(turn_speed);

    if (fabs(turn_speed) < fabs(prev_turn_speed)) decelerating = true;
}

bool TurnMode::hasSettled() {
    return angular_error_provider->hasSettled(target_heading);
}

DriveSpeeds TurnMode::getSpeeds() {
    return { left_speed, right_speed };
}

};
