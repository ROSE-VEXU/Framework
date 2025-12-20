#ifndef PIPELINE_STAGES_H
#define PIPELINE_STAGES_H

#include "DriveSpeedProvider.h"
#include "PositionProvider.h"

namespace BlackMagic {

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
    virtual void updateTarget(Position targetPosition, float targetHeading) = 0;
    virtual void update(Position currentPosition, float currentHeading) = 0;
    DriveSpeeds getSpeeds() override;
};

};

#endif