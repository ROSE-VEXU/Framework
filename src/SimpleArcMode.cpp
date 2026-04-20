#include "vex.h"

namespace BlackMagic {

SimpleArcMode::SimpleArcMode() {}

void SimpleArcMode::setTarget(float target_distance, Angle target_heading, ArcSettings arc_settings) {
    this->target_distance = target_distance * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
    this->target_heading = target_heading;
    this->arc_settings = arc_settings;
    this->linear_speed = 0;
    this->angular_speed = 0;
    this->settle_count = 0;
    this->settling_prev_left = 0;
    this->settling_prev_right = 0;
    this->settling_total_left = 0;
    this->settling_total_right = 0;
}

void SimpleArcMode::setErrorProviders(IErrorProvider& linear_error_provider, IErrorProvider& angular_error_provider) {
    this->linear_error_provider = &linear_error_provider;
    this->angular_error_provider = &angular_error_provider;
}


void SimpleArcMode::run(PID& linear_pid, PID& angular_pid) {
    float curr_linear_error = linear_error_provider->getError(target_distance);

    float pct_distance_traveled = linear_error_provider->getRawValue()/target_distance;
    float pct_to_mix_heading = BlackMagic::Utils::clamp((pct_distance_traveled-arc_settings.start_arc_pct)/arc_settings.arc_length_pct, 0.0f, 1.0f);

    float curr_heading_error = pct_to_mix_heading * angular_error_provider->getError(target_heading);

    linear_speed = linear_pid.getNextValue(curr_linear_error);
    angular_speed = angular_pid.getNextValue(curr_heading_error);
}

bool SimpleArcMode::hasSettled() {
    return linear_error_provider->hasSettled(target_distance) && angular_error_provider->hasSettled(target_heading);
}

DriveSpeeds SimpleArcMode::getSpeeds() {
    return BlackMagic::Utils::desaturateSpeeds(linear_speed, angular_speed);
}

};
