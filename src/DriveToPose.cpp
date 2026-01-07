#include "vex.h"

const float drift_comp = 0.25;
const float lead_pct = 0.25;
const float decel_linear_speed_limit = 47.0;
const float decel_start_distance = 360.0;

DriveToPose::DriveToPose() {}

void DriveToPose::updateTarget(BlackMagic::Pose target_pose) {
    this->target_pose = target_pose.rawForTrig();
    this->prev_same_side = true;
    this->prev_linear_speed = 0.0;
    this->max_linear_speed = 100.0; // Set to whatever we want to be the max speed

    this->decelerating = false;
    this->settle_count = 0;
    this->settling_prev_left = 0;
    this->settling_prev_right = 0;
    this->settling_total_left = 0;
    this->settling_total_right = 0;
}

void DriveToPose::update(BlackMagic::Pose curr_pose, const BlackMagic::DrivetrainState& drive_state, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) {
    this->curr_pose = curr_pose.rawForTrig();

    // Check if we're within decel range of the target position
    if (curr_pose.distanceTo(target_pose) < decel_start_distance && !decelerating) {
        decelerating = true;
        max_linear_speed = fmax(fabs(prev_linear_speed), decel_linear_speed_limit);
    }

    lookahead_position = getLookaheadPosition(curr_pose, target_pose);

    // Compute errors and speeds
    float linear_error = getLinearError(target_pose, lookahead_position, curr_pose);
    BlackMagic::Angle angular_error = getAngularError(target_pose, lookahead_position, curr_pose);
    float angular_speed = getAngularSpeed(angular_error.asDegrees(), angular_pid);
    float linear_speed = getLinearSpeed(linear_error, linear_pid, angular_speed, curr_pose, lookahead_position);
    prev_linear_speed = linear_speed;

    BlackMagic::DriveSpeeds scaledSpeeds = getScaledSpeedsFromMax(linear_speed, angular_speed);
    leftSpeed = scaledSpeeds.left;
    rightSpeed = scaledSpeeds.right;

    printf("target X: %.2f, target Y: %.2f, Heading: %.2f\n", target_pose.position.x, target_pose.position.y, target_pose.heading);
    printf("curr X: %.2f, curr Y: %.2f, Heading: %.2f\n", curr_pose.position.x, curr_pose.position.y, curr_pose.heading.asDegrees());
    printf("lookahead x: %.2f, lookahead y: %.2f\n", lookahead_position.x, lookahead_position.y);
    printf("linear error: %.2f, angular error: %.2f\n", linear_error, angular_error);
    // printf("linear speed: %.2f, angular speed: %.2f\n", linear_speed, angular_speed);
    // printf("left speed: %.2f, right speed: %.2f\n", leftSpeed, rightSpeed);
}

BlackMagic::DriveSpeeds DriveToPose::getScaledSpeedsFromMax(float linear_speed, float angular_speed) {
    float total_speed = fabs(linear_speed) + fabs(angular_speed);
    float left_speed = linear_speed + angular_speed;
    float right_speed = linear_speed - angular_speed;
    if (total_speed > 100.0) return { 100.0f * (left_speed/total_speed), 100.0f * (right_speed/total_speed) };
    return { left_speed, right_speed };
}

BlackMagic::Position DriveToPose::getLookaheadPosition(BlackMagic::Pose curr_pose, BlackMagic::Pose target_pose) {
    return twoPhaseControl(
        // Pre-Deceleration
        [&curr_pose, target_pose]() {
            float lookahead_x = lead_pct * curr_pose.distanceTo(target_pose) * cos(target_pose.heading.asRadians());
            float lookahead_y = lead_pct * curr_pose.distanceTo(target_pose) * sin(target_pose.heading.asRadians());
            BlackMagic::Position polar_lookahead = { lookahead_x, lookahead_y };
            return BlackMagic::Position{ target_pose.position.x - polar_lookahead.x, target_pose.position.y - polar_lookahead.y };
        },
        // Decelerating
        [target_pose]() {
            return target_pose.position;
        }
    );
}

float DriveToPose::getLinearError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose) {
    float distance_error = curr_pose.distanceTo(target_pose);
    // Global angle the robot must face to point at the lookahead point
    BlackMagic::Angle angle_to_lookahead = { atan2(lookahead_position.y - curr_pose.position.y, lookahead_position.x - curr_pose.position.x), BlackMagic::Angle::Unit::RAD };
    BlackMagic::Angle angle_error = BlackMagic::Utils::getAngleError(curr_pose.heading, angle_to_lookahead);
    // Scale speed; 0 when 90 degrees off target to turn to target, 1 when facing target, -1 when facing >90 deg away to back up towards target
    float speed_scale = cos(angle_error.asRadians());

    return twoPhaseControl(
        // Pre-Deceleration
        [distance_error, speed_scale]() {
            return distance_error * BlackMagic::Utils::sign(speed_scale);
        },
        // Decelerating
        [distance_error, speed_scale]() {
            return distance_error * speed_scale;
        }
    );
}

BlackMagic::Angle DriveToPose::getAngularError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose) {
    // Shortest angles are negated because we want the angle ERROR, meaning 20 deg right turn means we're 20 deg left of the target.
    return twoPhaseControl(
        // Pre-Deceleration
        [curr_pose, lookahead_position, this]() {
            BlackMagic::Angle angle_to_lookahead = { atan2(lookahead_position.y - curr_pose.position.y, lookahead_position.x - curr_pose.position.x), BlackMagic::Angle::Unit::RAD };
            printf("angle to lookahead rad: %.4f\n", angle_to_lookahead);
            printf("angle to lookahead deg: %.4f\n", angle_to_lookahead.asDegrees());
            return BlackMagic::Utils::getAngleError(curr_pose.heading, angle_to_lookahead);
        },
        // Decelerating
        [curr_pose, target_pose]() {
            return BlackMagic::Utils::getAngleError(curr_pose.heading, target_pose.heading);
        }
    );
}

float DriveToPose::getLinearSpeed(float linear_error, std::shared_ptr<BlackMagic::PID> linear_pid, float angular_speed, BlackMagic::Pose curr_pose, BlackMagic::Position lookahead_position) {
    float linear_speed = linear_pid->getNextValue(linear_error);
    linear_speed = BlackMagic::Utils::clamp(linear_speed, -max_linear_speed, max_linear_speed);

    // Limit speed to avoid drifting
    float curve_radius = 1/fabs(getSignedTangentArcCurvature(curr_pose, lookahead_position));
    curve_radius *= (WHEEL_DIAM_INCHES*M_PI)/360.0f; // Convert to inches
    curve_radius *= 1.0f/39.3701f; // Convert to m to be compatible with gravitational constant
    // Max speed derived from v^2/r <= mu * g when friction is greater than acceleration (friction not broken & we don't drift)
    // Thus, drift_comp is mu
    float max_speed_before_drifting = sqrt(drift_comp * 9.8f * curve_radius); // check b/c lem uses rad in meters
    linear_speed = BlackMagic::Utils::clamp(linear_speed, -max_speed_before_drifting, max_speed_before_drifting);

    // Limit linear movement if total speed will be above max
    float linear_reduction = fabs(angular_speed) + fabs(linear_speed) - max_linear_speed;
    if (linear_reduction > 0) linear_speed -= (linear_speed > 0 ? linear_reduction : -linear_reduction);

    // Never go backwards unless we're cleaning up our arrival position
    // linear_speed = fmax(0.0, linear_speed);linear_speed

    return linear_speed;
}

float DriveToPose::getSignedTangentArcCurvature(BlackMagic::Pose start_pose, BlackMagic::Position end_position) {
    // whether the pose is on the left or right side of the arc
    BlackMagic::Position delta = { end_position.x - start_pose.position.x, end_position.y - start_pose.position.y };
    // Positive for right turns, negative for left turns
    float robot_orientation_delta_cross_product = sin(start_pose.heading.asRadians()) * delta.x - cos(start_pose.heading.asRadians()) * delta.y;
    int arc_direction = BlackMagic::Utils::sign(robot_orientation_delta_cross_product);
    // calculate center point and radius, implicit B for standard form is -1
    float std_form_A = -tan(start_pose.heading.asRadians()); // slope of heading line * B
    float y_intercept = (tan(start_pose.heading.asRadians())*start_pose.position.x - start_pose.position.y); // y-intercept of heading line * B
    float perp_distance_from_end_to_heading_line = fabs(std_form_A * end_position.x + end_position.y + y_intercept) / sqrt(std_form_A * std_form_A + 1);
    float chord_length = start_pose.position.distanceTo(end_position);
    // return the curvature
    return arc_direction * ((2 * perp_distance_from_end_to_heading_line) / (chord_length * chord_length));
}

float DriveToPose::getAngularSpeed(float angular_error, std::shared_ptr<BlackMagic::PID> angular_pid) {
    return angular_pid->getNextValue(angular_error);
}

bool DriveToPose::hasSettled(const BlackMagic::DrivetrainState& drive_state) {
    // Evaluate for passing the target position via imaginary line
    bool robot_side = (curr_pose.position.y - target_pose.position.y) * -sin(target_pose.heading) <=
                      (curr_pose.position.x - target_pose.position.x) * cos(target_pose.heading);
    bool lookahead_side = (lookahead_position.y - target_pose.position.y) * -sin(target_pose.heading) <=
                          (lookahead_position.x - target_pose.position.x) * cos(target_pose.heading);
    bool same_side = robot_side == lookahead_side;
    // exit if close
    if (!same_side && prev_same_side && decelerating) return true;
    else {
        prev_same_side = same_side;

        if (!decelerating) return false;

        float curr_left = drive_state.leftDegrees;
        float curr_right = drive_state.rightDegrees;

        settling_total_left += fabs(curr_left - settling_prev_left);
        settling_total_right += fabs(curr_right - settling_prev_right);

        if (settling_total_left < PIPELINE_DRIVE_SETTLE_DEG_THRESHOLD &&
            settling_total_right < PIPELINE_DRIVE_SETTLE_DEG_THRESHOLD) {
            settle_count++;
        } else {
            settle_count = 0;
            settling_total_left = 0;
            settling_total_right = 0;
        }
        settling_prev_left = curr_left;
        settling_prev_right = curr_right;

        return (settle_count > PIPELINE_DRIVE_SETTLE_COUNT) ? true : false;
    }
}

BlackMagic::DriveSpeeds DriveToPose::getSpeeds() {
    return { leftSpeed, rightSpeed };
}
