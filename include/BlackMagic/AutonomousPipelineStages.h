#ifndef PIPELINE_STAGES_H
#define PIPELINE_STAGES_H

#include "PID.h"
#include "DriveSpeedProvider.h"
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

class Pose {
public:
    BlackMagic::Position position;
    float heading;

    float distanceTo(Pose other) {
        return position.distanceTo(other.position);
    };
};

class ISpeedController: IDriveSpeedProvider {
public:
    // use updateTarget to do things like generate a new path
    virtual void updateTarget(Pose target_pose) = 0;
    virtual void update(Pose curr_pose, const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) = 0;
    virtual bool hasSettled(const DriveModeUtilFunctions& utils) = 0;
    DriveSpeeds getSpeeds() override;
};

};

#endif