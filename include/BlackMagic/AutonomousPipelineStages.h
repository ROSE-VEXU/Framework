#ifndef PIPELINE_STAGES_H
#define PIPELINE_STAGES_H

#include "DriveSpeedProvider.h"
#include "DriveStates.h"
#include "PID.h"
#include "PositionProvider.h"
#include <memory>

namespace BlackMagic {

struct DriveModeUtilFunctions;

class IOdometryPipelineStage: IPositionProvider {
public:
    virtual void update() = 0;
    virtual void calibrate() = 0;
    Position getPosition() override;
};

class ILocalizationPipelineStage: IPositionProvider {
public:
    virtual void update() = 0;
    Position getPosition() override;
};

class ISpeedController: IDriveSpeedProvider {
public:
    // use updateTarget to do things like generate a new path
    virtual void updateTarget(Pose target_pose) = 0;
    virtual void update(Pose curr_pose, const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) = 0;
    virtual bool hasSettled(const DrivetrainState& drive_state) = 0;
    DriveSpeeds getSpeeds() override;
};

};

#endif