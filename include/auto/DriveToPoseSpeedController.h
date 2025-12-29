#include "./BlackMagic/AutonomousPipelineStages.h"
#include "./BlackMagic/PositionProvider.h"
#include "./BlackMagic/DriveSpeedProvider.h"
#include <functional>

class DriveToPoseSpeedController: public BlackMagic::ISpeedController {
public:
    DriveToPoseSpeedController();
    void updateTarget(BlackMagic::Pose target_pose) override;
    void update(BlackMagic::Pose curr_pose, const BlackMagic::DriveModeUtilFunctions& utils, std::shared_ptr<BlackMagic::PID> linear_pid, std::shared_ptr<BlackMagic::PID> angular_pid) override;
    bool hasSettled(const BlackMagic::DriveModeUtilFunctions& utils) override;
    BlackMagic::DriveSpeeds getSpeeds() override;
private:
    BlackMagic::Pose target_pose;
    BlackMagic::Pose curr_pose;
    BlackMagic::Position lookahead_position;
    bool decelerating;
    bool prev_same_side;
    float decel_start_distance;
    float decel_linear_speed_limit;
    float max_linear_speed;
    float prev_linear_speed;

    float leftSpeed;
    float rightSpeed;

    template<typename Phase1, typename Phase2>
    auto twoPhaseControl(Phase1 phase_one_action, Phase2 phase_two_action) -> decltype(phase_one_action()) {
        if (this->decelerating) {
            return phase_two_action();
        } else {
            return phase_one_action();
        }
    };

    BlackMagic::DriveSpeeds getScaledSpeedsFromMax(float linear_speed, float angular_speed);
    BlackMagic::Position getLookaheadPosition(BlackMagic::Pose curr_pose, BlackMagic::Pose target_pose);
    float getLinearError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose);
    float getAngularError(BlackMagic::Pose target_pose, BlackMagic::Position lookahead_position, BlackMagic::Pose curr_pose);
    float getAngularSpeed(float angular_error, std::shared_ptr<BlackMagic::PID> angular_pid);
    float getLinearSpeed(float linear_error, std::shared_ptr<BlackMagic::PID> linear_pid, float angular_speed, BlackMagic::Pose curr_pose, BlackMagic::Position lookahead_position);
    float getSignedTangentArcCurvature(BlackMagic::Pose start_pose, BlackMagic::Position end_position);
};