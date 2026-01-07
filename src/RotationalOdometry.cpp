#include "vex.h"
#include <algorithm>

RotationalOdometry::RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, vex::inertial imu_1, vex::inertial imu_2, RotationalOdometryConfig&& config):
    vert_tracker(vert_tracker),
    hori_tracker(hori_tracker),
    imu_1(imu_1),
    imu_2(imu_2),
    xPosition(0.0),
    yPosition(0.0),
    config(std::move(config)),
    prev_state() {
    calibrate();
}

void RotationalOdometry::calibrate() {
    vert_tracker.resetPosition();
    hori_tracker.resetPosition();
    imu_1.calibrate();
    imu_2.calibrate();
    xPosition = 0.0;
    yPosition = 0.0;
}

float RotationalOdometry::getHeading() {
    float imu_1_heading = imu_1.heading(vex::rotationUnits::deg);
    float imu_2_heading = imu_2.heading(vex::rotationUnits::deg);
    float max = std::max(imu_1_heading, imu_2_heading);
    float min = std::min(imu_1_heading, imu_2_heading);

    if (max - min > 180) { // Massive difference between the two means that they're on opposite sides of 360
        max -= 180;
        min += 180;
        return fmod(((max+min) / 2.0) + 180.0, 360.0); // Mod is necessary to prevent returning > 360
    }

    return ((max+min) / 2.0);
}

float RotationalOdometry::toRadians(float degrees) {
    return degrees * (M_PI/180.0);
}

// TODO - Put vert, hori, heading into struct?
void RotationalOdometry::update() {
    // Get new values
    RotationalOdometryState state;
    state.vert = vert_tracker.position(vex::rotationUnits::deg);
    state.hori = hori_tracker.position(vex::rotationUnits::deg);
    state.heading = toRadians(getHeading());

    // Compute deltas
    float delta_vert = state.vert-prev_state.vert;
    float delta_hori = state.hori-prev_state.hori;
    float delta_heading = state.heading-prev_state.heading;

    float vert_chord_length = (delta_heading == 0.0) ? delta_vert
                                                     : (2.0 * ((delta_vert/delta_heading) + config.vert_tracker_offset) * sin(delta_heading/2.0));

    float hori_chord_length = (delta_heading == 0.0) ? delta_hori
    // -2 is offset on hori tracking wheel
                                                     : 2.0 * ((delta_hori/delta_heading) + config.hori_tracker_offset) * sin(delta_heading/2.0);

    float local_polar_angle;
    float local_polar_length;

    if (hori_chord_length == 0 && vert_chord_length == 0){
        local_polar_angle = 0;
        local_polar_length = 0;
    } else {
        local_polar_angle = atan2(vert_chord_length, hori_chord_length);
        local_polar_length = hypot(hori_chord_length, vert_chord_length); 
    }

    float avg_heading = prev_state.heading + (delta_heading/2);
    float global_polar_angle = local_polar_angle - avg_heading;

    float x_position_delta = local_polar_length*cos(global_polar_angle); 
    float y_position_delta = local_polar_length*sin(global_polar_angle);

    xPosition += x_position_delta;
    yPosition += y_position_delta;

    // Update values
    prev_state.vert = state.vert;
    prev_state.hori = state.hori;
    prev_state.heading = state.heading;
}

float RotationalOdometry::getX() {
    return xPosition/360.0 * M_PI * WHEEL_DIAM_INCHES;
}

float RotationalOdometry::getY() {
    return yPosition/360.0 * M_PI * WHEEL_DIAM_INCHES;
}
