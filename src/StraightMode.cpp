#include "vex.h"

namespace BlackMagic {

StraightMode::StraightMode() {}

void StraightMode::setTarget(float target_distance, Angle target_heading) {
    this->target_distance = target_distance;
    this->target_heading = target_heading;
    this->linear_speed = 0;
    this->angular_speed = 0;
    this->settle_count = 0;
    this->settling_prev_left = 0;
    this->settling_prev_right = 0;
    this->settling_total_left = 0;
    this->settling_total_right = 0;
}

void StraightMode::setErrorProviders(IErrorProvider& linear_error_provider, IErrorProvider& angular_error_provider) {
    this->linear_error_provider = &linear_error_provider;
    this->angular_error_provider = &angular_error_provider;
}

void StraightMode::run(PID& linear_pid, PID& angular_pid) {
    float curr_linear_error = linear_error_provider->getError(target_distance);
    float curr_angular_error = angular_error_provider->getError(target_heading);

    linear_speed = linear_pid.getNextValue(curr_linear_error);
    angular_speed = angular_pid.getNextValue(curr_angular_error);
}

bool StraightMode::hasSettled() {
    return linear_error_provider->hasSettled(target_distance) && angular_error_provider->hasSettled(target_heading);
}

DriveSpeeds StraightMode::getSpeeds() {
    return BlackMagic::Utils::desaturateSpeeds(linear_speed, angular_speed);
}

};
