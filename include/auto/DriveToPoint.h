#ifndef DRIVE_TO_POINT_SPEED_CONTROLLER_H
#define DRIVE_TO_POINT_SPEED_CONTROLLER_H

#include "../BlackMagic/AutonomousPipelineStages.h"
#include "../BlackMagic/PositionProvider.h"
#include "../BlackMagic/DriveSpeedProvider.h"
#include <functional>

class DriveToPoint: public BlackMagic::ISpeedController {
public:
    enum Phase {
        FACE_POINT,
        DRIVE_TO_POINT,
        FACE_TARGET
    };

    DriveToPoint();
    void updateTarget(BlackMagic::Pose target_pose) override;
    void update(BlackMagic::Pose curr_pose, const BlackMagic::DrivetrainState& drive_state, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) override;
    bool hasSettled(const BlackMagic::DrivetrainState& drive_state) override;
    BlackMagic::DriveSpeeds getSpeeds() override;
private:
    struct PhaseTarget {
        float distance;
        BlackMagic::Angle heading;
    };

    struct PhaseTargets {
        PhaseTarget face_point_heading;
        PhaseTarget drive_to_point_degrees;
        PhaseTarget face_target_heading;
    };

    BlackMagic::Pose target_pose;
    PhaseTargets target_values;

    Phase curr_phase;
    Phase prev_phase;
    BlackMagic::DriveSpeeds speeds;
    BlackMagic::DriveSpeeds prev_speeds;

    bool decelerating;

    int settle_count;
    float settling_prev_left;
    float settling_prev_right;
    float settling_total_left;
    float settling_total_right;

    template<typename Phase1, typename Phase2, typename Phase3>
    auto threePhaseControl(Phase1 phase_one_action, Phase2 phase_two_action, Phase3 phase_three_action) -> decltype(phase_one_action()) {
        switch (curr_phase) {
            case Phase::FACE_POINT:
                return phase_one_action();
            case Phase::DRIVE_TO_POINT:
                return phase_two_action();
            case Phase::FACE_TARGET:
                return phase_three_action();
        }
    };

    // PhaseTargets computePhaseTargets();
    // PhaseTarget getPhaseTarget();

    bool hasPhaseSettled();
};

#endif