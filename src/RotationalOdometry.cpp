#include "vex.h"
#include <algorithm>

RotationalOdometry::RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, BlackMagic::IHeadingProvider& heading_provider, RotationalOdometryConfig&& config):
    vert_tracker(vert_tracker),
    hori_tracker(hori_tracker),
    heading_provider(heading_provider),
    rawPosition({ 0.0, 0.0 }),
    config(std::move(config)),
    prev_state() {
    calibrate();
}

void RotationalOdometry::calibrate() {
    vert_tracker.resetPosition();
    hori_tracker.resetPosition();
    heading_provider.calibrate();
    rawPosition = { 0.0, 0.0 };
}

void RotationalOdometry::update() {
    // Get new values
    RotationalOdometryState state;
    state.vert = vert_tracker.position(vex::rotationUnits::deg);
    state.hori = hori_tracker.position(vex::rotationUnits::deg);
    state.heading = heading_provider.getHeading().asRadians();

    // Compute deltas
    float delta_vert = state.vert-prev_state.vert;
    delta_vert = (delta_vert/360.0f) * (M_PI*ODOM_WHEEL_DIAM_INCHES);
    float delta_hori = state.hori-prev_state.hori;
    delta_hori = (delta_hori/360.0f) * (M_PI*ODOM_WHEEL_DIAM_INCHES);
    float delta_heading = std::remainder(state.heading-prev_state.heading, 2.0f*M_PI);

    float vert_chord_length = (fabs(delta_heading) < 1e-6f) ? delta_vert
                                                     : (2.0 * ((delta_vert/delta_heading) + config.vert_tracker_offset) * sin(delta_heading/2.0));

    float hori_chord_length = (fabs(delta_heading) < 1e-6f) ? delta_hori
                                                     : 2.0 * ((delta_hori/delta_heading) + config.hori_tracker_offset) * sin(delta_heading/2.0);

    float local_polar_angle;
    float local_polar_length;

    if (hori_chord_length == 0 && vert_chord_length == 0){
        local_polar_angle = 0;
        local_polar_length = 0;
    } else {
        local_polar_angle = atan2(vert_chord_length, hori_chord_length); // (y, x)
        local_polar_length = hypot(vert_chord_length, hori_chord_length); 
    }

    float global_polar_angle = local_polar_angle - prev_state.heading - (delta_heading/2);

    float x_position_delta = local_polar_length*cos(global_polar_angle); 
    float y_position_delta = local_polar_length*sin(global_polar_angle);

    rawPosition.x += x_position_delta;
    rawPosition.y += y_position_delta;

    // Update values
    prev_state.vert = state.vert;
    prev_state.hori = state.hori;
    prev_state.heading = state.heading;
}

BlackMagic::Position RotationalOdometry::getPosition() {
    return rawPosition;
}
