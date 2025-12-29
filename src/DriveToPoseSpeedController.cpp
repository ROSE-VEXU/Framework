#include "vex.h"

const float drift_comp = 8.0;
const float lead_pct = 0.25;
const float early_exit_threshold = 0.5;

DriveToPoseSpeedController::DriveToPoseSpeedController() {}

void DriveToPoseSpeedController::updateTarget(BlackMagic::Pose target_pose) {
    this->target_pose = target_pose;
    this->decelerating = false;
    this->prev_same_side = true;
    this->prev_linear_speed = 0.0;
    this->max_linear_speed = 100.0; // Set to whatever we want to be the max speed
}

void DriveToPoseSpeedController::update(BlackMagic::Pose curr_pose, const BlackMagic::DriveModeUtilFunctions& utils, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) {
    this->curr_pose = curr_pose;
    
    // Check if we're within decel range of the target position
    if (curr_pose.distanceTo(target_pose) < decel_start_distance && !decelerating) {
        decelerating = true;
        max_linear_speed = fmax(fabs(prev_linear_speed), decel_linear_speed_limit);
    }

    lookahead_position = getLookaheadPosition(curr_pose, target_pose);

    // Compute errors and speeds
    float linear_error = getLinearError(target_pose, lookahead_position, curr_pose);
    float angular_error = getAngularError(target_pose, lookahead_position, curr_pose);
    float angular_speed = getAngularSpeed(angular_error, angular_pid);
    float linear_speed = getLinearSpeed(linear_error, linear_pid, angular_speed, curr_pose, lookahead_position);

    BlackMagic::DriveSpeeds scaledSpeeds = getScaledSpeedsFromMax(linear_speed, angular_speed);
    leftSpeed = scaledSpeeds.left;
    rightSpeed = scaledSpeeds.right;
}

BlackMagic::DriveSpeeds DriveToPoseSpeedController::getScaledSpeedsFromMax(float linear_speed, float angular_speed) {
    float totalSpeed = fabs(linear_speed) + fabs(angular_speed);
    float leftSpeed = linear_speed - angular_speed;
    float rightSpeed = linear_speed + angular_speed;
    if (totalSpeed > 100.0) return { .left = leftSpeed / totalSpeed, .right = rightSpeed / totalSpeed };
    return { .left = leftSpeed, .right = rightSpeed };
}

BlackMagic::Position DriveToPoseSpeedController::getLookaheadPosition(BlackMagic::Pose curr_pose, BlackMagic::Pose target_pose) {
    return twoPhaseControl(
        // Pre-Deceleration
        [&curr_pose, target_pose]() {
            float lookahead_x = lead_pct * curr_pose.distanceTo(target_pose) * cos(target_pose.heading);
            float lookahead_y = lead_pct * curr_pose.distanceTo(target_pose) * sin(target_pose.heading);
            BlackMagic::Position polar_lookahead = { lookahead_x, lookahead_y };
            return BlackMagic::Position{ target_pose.position.x - polar_lookahead.x, target_pose.position.y - polar_lookahead.y };
        },
        // Decelerating
        [target_pose]() {
            return target_pose.position;
        }
    );
}

float DriveToPoseSpeedController::getLinearError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose) {
    float distance_error = curr_pose.distanceTo(target_pose);
    // Global angle the robot must face to point at the lookahead point
    float angle_to_lookahead = atan2(lookahead_position.y - curr_pose.position.y, lookahead_position.x - curr_pose.position.x);
    float angle_error = curr_pose.heading - angle_to_lookahead;
    // Scale speed; 0 when 90 degrees off target to turn to target, 1 when facing target, -1 when facing >90 deg away to back up towards target
    float speed_scale = cos(angle_error);

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

float DriveToPoseSpeedController::getAngularError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose) {
    return twoPhaseControl(
        // Pre-Deceleration
        [curr_pose, lookahead_position]() {
            float angle_to_lookahead = atan2(lookahead_position.y - curr_pose.position.y, lookahead_position.x - curr_pose.position.x);
            return curr_pose.heading - angle_to_lookahead;
        },
        // Decelerating
        [curr_pose, target_pose]() {
            return curr_pose.heading - target_pose.heading;
        }
    );
}

float DriveToPoseSpeedController::getLinearSpeed(float linear_error, std::shared_ptr<BlackMagic::PID> linear_pid, float angular_speed, BlackMagic::Pose curr_pose, BlackMagic::Position lookahead_position) {
    float linear_speed = linear_pid->getNextValue(linear_error);
    linear_speed = fmax(fmin(linear_speed, max_linear_speed), -max_linear_speed);

    // Limit speed to avoid drifting
    float curve_radius = 1/fabs(getSignedTangentArcCurvature(curr_pose, lookahead_position));
    // Max speed derived from v^2/r <= mu * g when friction is greater than acceleration (friction not broken & we don't drift)
    float max_speed_before_drifting = sqrt(drift_comp * curve_radius);
    linear_speed = fmin(linear_speed, max_speed_before_drifting);

    // Limit linear movement if total speed will be above max
    float linear_reduction = fabs(linear_speed) + fabs(angular_speed) - max_linear_speed;
    linear_speed -= linear_reduction;

    // Never go backwards unless we're cleaning up our arrival position
    if (!decelerating) {
        linear_speed = fmax(0.0, linear_speed);
    }

    return linear_speed;
}

float DriveToPoseSpeedController::getSignedTangentArcCurvature(BlackMagic::Pose start_pose, BlackMagic::Position end_position) {
    // whether the pose is on the left or right side of the arc
    BlackMagic::Position delta = { end_position.x - start_pose.position.x, end_position.y - start_pose.position.y };
    // Positive for right turns, negative for left turns
    float robot_orientation_delta_cross_product = sin(start_pose.heading) * delta.x - cos(start_pose.heading) * delta.y;
    int arc_direction = BlackMagic::Utils::sign(robot_orientation_delta_cross_product);
    // calculate center point and radius
    float std_form_B = -1;
    float std_form_A = tan(start_pose.heading) * std_form_B; // slope of heading line * B
    float y_intercept = (start_pose.position.y - tan(start_pose.heading)*start_pose.position.x) * std_form_B; // y-intercept of heading line * B
    float perp_distance_from_end_to_heading_line = fabs(std_form_A * end_position.x + end_position.y + y_intercept) / sqrt(std_form_A * std_form_A + 1);
    float chord_length = start_pose.position.distanceTo(end_position);
    // return the curvature
    return arc_direction * ((2 * perp_distance_from_end_to_heading_line) / (chord_length * chord_length));
}

float DriveToPoseSpeedController::getAngularSpeed(float angular_error, std::shared_ptr<BlackMagic::PID> angular_pid) {
    return angular_pid->getNextValue(angular_error);
}

bool DriveToPoseSpeedController::hasSettled(const BlackMagic::DriveModeUtilFunctions& utils) {
    // Evaluate for passing the target position via imaginary line
    bool robot_side = (curr_pose.position.y - target_pose.position.y) * -sin(target_pose.heading) <=
                             (curr_pose.position.x - target_pose.position.x) * cos(target_pose.heading) + early_exit_threshold;
    bool lookahead_side = (lookahead_position.y - target_pose.position.y) * -sin(target_pose.heading) <=
                                 (lookahead_position.x - target_pose.position.x) * cos(target_pose.heading) + early_exit_threshold;
    bool same_side = robot_side == lookahead_side;
    // exit if close
    if (!same_side && prev_same_side && decelerating) return true;
    prev_same_side = same_side;

    return false;
}

BlackMagic::DriveSpeeds DriveToPoseSpeedController::getSpeeds() {
    return { leftSpeed, rightSpeed };
}
