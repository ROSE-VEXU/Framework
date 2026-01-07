#include "vex.h"

SimpleDriveToPoint::SimpleDriveToPoint() {}

void SimpleDriveToPoint::updateTarget(BlackMagic::Pose target_pose) {
    this->target_pose = target_pose.rawForTrig();
    this->curr_phase = SimpleDriveToPoint::Phase::FACE_POINT;
    this->speeds = { 0.0, 0.0 };
    this->prev_speeds = { 0.0, 0.0 };
    this->angular_speed = 0.0;
    this->prev_angular_speed = 0.0;
    this->face_target_heading = 0.0_deg;
    this->decelerating = false;
    this->needs_compute = true;
    this->reset = true;
}

void SimpleDriveToPoint::update(BlackMagic::Pose curr_pose, const BlackMagic::DrivetrainState& drive_state, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) {
    curr_pose = curr_pose.rawForTrig();
    
    if (decelerating) hasPhaseSettled(drive_state); // Update settle state to check for phase changes
    if (needs_compute) {
        float cartesian_target_angle = curr_pose.angleTo(target_pose);
        face_target_heading = { fmod((M_TWOPI - cartesian_target_angle + M_PI/2.0f), M_TWOPI), BlackMagic::Angle::Unit::RAD };
        printf("Target Heading: %.2f\n", face_target_heading.asDegrees());
        needs_compute = false;
    }
    if (reset) {
        linear_pid->reset();
        angular_pid->reset();
        this->speeds = { 0.0, 0.0 };
        this->prev_speeds = { 0.0, 0.0 };
        this->angular_speed = 0.0;
        this->prev_angular_speed = 0.0;
        reset = false;
    }

    float linear_err;
    float angular_err;

    threePhaseControl(
        // FACE_POINT
        [this, &curr_pose, &linear_err, &angular_err]() {
            linear_err = 0.0;
            angular_err = BlackMagic::Utils::getShortestAngleBetween(curr_pose.heading, face_target_heading);
            printf("Angular Error: %.2f\n", BlackMagic::Utils::getShortestAngleBetween(curr_pose.heading, face_target_heading));
        },
        // DRIVE_TO_POINT
        [this, &curr_pose, &linear_err, &angular_err]() {
            float delta_x = target_pose.position.x - curr_pose.position.x;
            float delta_y = target_pose.position.y - curr_pose.position.y;

            float radian_heading = curr_pose.heading.asRadians();
            radian_heading = fmod((M_TWOPI - (radian_heading-M_PI/2.0f)), M_TWOPI);
            float signed_distance = delta_x * cos(radian_heading) + delta_y * sin(radian_heading);

            printf("X: %.2f Y: %.2f, Distance: %.2f\n", curr_pose.position.x, curr_pose.position.y, signed_distance);
            linear_err = 0.0;//signed_distance;
            angular_err = 0.0;;//BlackMagic::Utils::getShortestAngleBetween(curr_pose.heading, face_target_heading);
        },
        // FACE_TARGET
        [this, &curr_pose, &linear_err, &angular_err]() {
            printf("face target\n");
            linear_err = 0.0;
            angular_err = 0.0;//BlackMagic::Utils::getShortestAngleBetween(curr_pose.heading, target_pose.heading);
        }
    );

    float linear_speed = linear_pid->getNextValue(linear_err);
    prev_angular_speed = angular_speed;
    angular_speed = angular_pid->getNextValue(angular_err);
    prev_speeds = speeds;
    speeds = BlackMagic::Utils::getScaledSpeedsFromMax(linear_speed, angular_speed);

    decelerating = threePhaseControl(
        // FACE_POINT
        [this, &curr_pose, &linear_err, &angular_err]() {
            if (fabs(angular_speed) < fabs(prev_angular_speed) &&
                !decelerating) {
                return true;
            } else return false;
        },
        // DRIVE_TO_POINT
        [this, &curr_pose, &linear_err, &angular_err]() {
            if (fabs(speeds.left) < fabs(prev_speeds.left) &&
                fabs(speeds.right) < fabs(prev_speeds.right) &&
                !decelerating) {
                return true;
            } else return false;
        },
        // FACE_TARGET
        [this, &curr_pose, &linear_err, &angular_err]() {
            if (fabs(angular_speed) < fabs(prev_angular_speed) &&
                !decelerating) {
                return true;
            } else return false;
        }
    );
}


bool SimpleDriveToPoint::hasSettled(const BlackMagic::DrivetrainState& drive_state) {
    return (curr_phase == SimpleDriveToPoint::Phase::FACE_TARGET) && hasPhaseSettled(drive_state);
}

bool SimpleDriveToPoint::hasPhaseSettled(const BlackMagic::DrivetrainState& drive_state) {
    if (!decelerating) return false;

    return threePhaseControl(
        // FACE_POINT
        [this, &drive_state]() {
            float curr_heading = BlackMagic::Utils::getShortestAngleBetween(drive_state.heading, face_target_heading);
            settling_total_heading += fabs(curr_heading - settling_prev_heading);

            if (fabs(settling_total_heading) < TURN_DRIVE_SETTLE_HEADING_THRESHOLD) {
                settle_count++;
            } else {
                settle_count = 0;
                settling_total_heading = 0;
            }
            settling_prev_heading = curr_heading;

            if (settle_count > TURN_DRIVE_SETTLE_COUNT) {
                decelerating = false;
                reset = true;
                settle_count = 0;
                settling_total_heading = 0;
                curr_phase = SimpleDriveToPoint::Phase::DRIVE_TO_POINT;
                return true;
            } else return false;
        },
        // DRIVE_TO_POINT
        [this, &drive_state]() {
            float curr_left = drive_state.leftDegrees;
            float curr_right = drive_state.rightDegrees;

            settling_total_left += fabs(curr_left - settling_prev_left);
            settling_total_right += fabs(curr_right - settling_prev_right);

            printf("Left total: %.2f, Right total: %.2f\n", settling_total_left, settling_total_right);

            if (settling_total_left < STRAIGHT_DRIVE_SETTLE_DEG_THRESHOLD &&
                settling_total_right < STRAIGHT_DRIVE_SETTLE_DEG_THRESHOLD) {
                settle_count++;
            } else {
                settle_count = 0;
                settling_total_left = 0;
                settling_total_right = 0;
            }
            settling_prev_left = curr_left;
            settling_prev_right = curr_right;

            if (settle_count > STRAIGHT_DRIVE_SETTLE_COUNT) {
                decelerating = false;
                reset = true;
                settle_count = 0;
                settling_total_left = 0;
                settling_total_right = 0;
                curr_phase = SimpleDriveToPoint::Phase::FACE_TARGET;
                return true;
            } else return false;
        },
        // FACE_TARGET
        [this, &drive_state]() {
            float curr_heading = BlackMagic::Utils::getShortestAngleBetween(drive_state.heading, target_pose.heading);
            settling_total_heading += fabs(curr_heading - settling_prev_heading);

            if (fabs(settling_total_heading) < TURN_DRIVE_SETTLE_HEADING_THRESHOLD) {
                settle_count++;
            } else {
                settle_count = 0;
                settling_total_heading = 0;
            }
            settling_prev_heading = curr_heading;

            return (settle_count > TURN_DRIVE_SETTLE_COUNT) ? true : false;

        }
    );
}

BlackMagic::DriveSpeeds SimpleDriveToPoint::getSpeeds() {
    return { 0.9f*speeds.left, 0.9f*speeds.right };
}
