#include "vex.h"

DriveToPoint::DriveToPoint() {}

void DriveToPoint::updateTarget(BlackMagic::Pose target_pose) {
    this->target_pose = target_pose.rawForTrig();
    this->curr_phase = DriveToPoint::Phase::DRIVE_TO_POINT;
    this->speeds = { 0.0, 0.0 };
    this->prev_speeds = { 0.0, 0.0 };
}

void DriveToPoint::update(BlackMagic::Pose curr_pose, const BlackMagic::DrivetrainState& drive_state, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) {
    curr_pose = curr_pose.rawForTrig();

    float curr_linear_error = curr_pose.distanceTo(target_pose);
    if (decelerating && curr_linear_error < STRAIGHT_DRIVE_SETTLE_DEG_THRESHOLD) {
        curr_phase = DriveToPoint::Phase::FACE_TARGET;
        decelerating = false;
    }

    BlackMagic::Angle target_heading;

    if (curr_phase == DriveToPoint::Phase::DRIVE_TO_POINT) {
        float cartesian_target_angle = curr_pose.angleTo(target_pose);
        target_heading = { fmod((M_TWOPI - cartesian_target_angle + M_PI/2.0f), M_TWOPI), BlackMagic::Angle::Unit::RAD };
    } else {
        target_heading = target_pose.heading;
    }
    float curr_angular_error = BlackMagic::Utils::getShortestAngleBetween(drive_state.heading, target_heading);

    printf("Linear Error: %.2f, Angular Error: %.2f\n", curr_linear_error, curr_angular_error);

    float angular_speed = angular_pid->getNextValue(curr_angular_error);
    float linear_speed = 0.0f;
    if (curr_phase == DriveToPoint::Phase::DRIVE_TO_POINT) {
        linear_speed = linear_pid->getNextValue(curr_linear_error);
    }

    prev_speeds = speeds;
    speeds = BlackMagic::Utils::getScaledSpeedsFromMax(linear_speed, angular_speed);

    if (fabs(speeds.left) < fabs(prev_speeds.left) &&
        fabs(speeds.right) < fabs(prev_speeds.right) &&
        !decelerating) {
        decelerating = true;
    }
}

// DriveToPoint::PhaseTargets DriveToPoint::computePhaseTargets() {
//     return {
//         { 0.0, { 0.0 , BlackMagic::Angle::Unit::DEG } },
//         { 0.0, { 0.0 , BlackMagic::Angle::Unit::DEG } },
//         { 0.0, { 0.0 , BlackMagic::Angle::Unit::DEG } }
//     };
// }

// DriveToPoint::PhaseTarget DriveToPoint::getPhaseTarget() {
//     threePhaseControl(
//         // FACE_POINT
//         [this]() {
//             return target_values.face_point_heading;
//         },
//         // DRIVE_TO_POINT
//         [this]() {
//             return target_values.drive_to_point_degrees;
//         },
//         // FACE_TARGET
//         [this]() {
//             return target_values.face_target_heading;
//         }
//     );

//     return { 0.0, 0.0 };
// }

bool DriveToPoint::hasSettled(const BlackMagic::DrivetrainState& drive_state) {
    return false;
    //return (curr_phase == DriveToPoint::Phase::FACE_TARGET) && hasPhaseSettled();
}

bool DriveToPoint::hasPhaseSettled() {
    if (!decelerating) return false;

    threePhaseControl(
        // FACE_POINT
        []() {

        },
        // DRIVE_TO_POINT
        []() {

        },
        // FACE_TARGET
        []() {

        }
    );

    return true;
}

BlackMagic::DriveSpeeds DriveToPoint::getSpeeds() {
    return { speeds.left, speeds.right };
}
