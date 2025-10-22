#include "vex.h"

// TODO - bad practice, fix this later
const float vert_wheel_offset = 0.0; // horizontal offset from the center of the robot in inches
const float hori_wheel_offset = 0.0; // vertical offset from the center of the robot in inches
float prev_vert = 0.0;
float prev_hori = 0.0;
float prev_heading = 0.0;

RotationalOdometry::RotationalOdometry(vex::rotation vert_tracker, vex::rotation hori_tracker, vex::inertial imu_1, vex::inertial imu_2):
    vert_tracker(vert_tracker),
    hori_tracker(hori_tracker),
    imu_1(imu_1),
    imu_2(imu_2),
    xPosition(0.0),
    yPosition(0.0) {
    calibrate();
}

// start TODO - add to class
void RotationalOdometry::calibrate() {
    vert_tracker.resetPosition();
    hori_tracker.resetPosition();
    imu_1.calibrate();
    imu_2.calibrate();
}

float toRadians(float deg) {
    return deg * (M_PI/180.0);
}

// end TODO - add to class

// TODO - Put vert, hori, heading into struct?
void RotationalOdometry::update() {
    // Get new values
    float vert = vert_tracker.position(vex::rotationUnits::deg);
    float hori = hori_tracker.position(vex::rotationUnits::deg);
    float heading = imu_1.heading();//(imu_1.heading() + imu_2.heading()) / 2.0;
    heading = toRadians(heading);

    // Compute deltas
    float delta_vert = vert-prev_vert;
    float delta_hori = hori-prev_hori;
    float delta_heading = heading-prev_heading;

    float vert_chord_length = (delta_heading == 0.0) ? delta_vert
                                                     : (2.0 * (delta_vert/delta_heading) * sin(delta_heading/2.0));
    float vert_delta_x = vert_chord_length * cos(heading);
    float vert_delta_y = vert_chord_length * sin(heading);

    float hori_heading = delta_heading;// + (M_PI/2.0);
    float hori_chord_length = (delta_heading == 0.0) ? delta_hori
    // -4.0 is the inch offset of the hori tracking wheel
                                                     : 2.0 * ((delta_hori/hori_heading) + -2.0) * sin(hori_heading/2.0);
    // float hori_chord_theta = (M_PI-hori_heading) / 2.0;
    float hori_delta_x = hori_chord_length * -sin(heading);
    float hori_delta_y = hori_chord_length * cos(heading);



    float local_polar_angle;
    float local_polar_length;

    if (hori_chord_length == 0 && vert_chord_length == 0){
        local_polar_angle = 0;
        local_polar_length = 0;
    } else {
        local_polar_angle = atan2(vert_chord_length, hori_chord_length); 
        local_polar_length = sqrt(pow(hori_chord_length, 2) + pow(vert_chord_length, 2)); 
    }

    float global_polar_angle = local_polar_angle - prev_heading - (delta_heading/2);

    float X_position_delta = local_polar_length*cos(global_polar_angle); 
    float Y_position_delta = local_polar_length*sin(global_polar_angle);

    // xPosition += vert_delta_x + hori_delta_x;
    // yPosition += vert_delta_y + hori_delta_y;
    xPosition += X_position_delta;
    yPosition += Y_position_delta;

    // Update values
    prev_vert = vert;
    prev_hori = hori;
    prev_heading = heading;
}

float RotationalOdometry::getX() {
    return xPosition/360.0 * 8.6393798;
}

float RotationalOdometry::getY() {
    return yPosition/360.0 * 8.6393798;
}
