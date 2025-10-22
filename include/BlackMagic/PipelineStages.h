#ifndef PIPELINE_STAGES_H
#define PIPELINE_STAGES_H

#include "DriveSpeedProvider.h"
#include "PositionProvider.h"

namespace BlackMagic {

class IOdometryPipelineStage: IPositionProvider {
public:
    virtual void update() = 0;
    virtual void calibrate() = 0;
    float getX() override;
    float getY() override;
};

class ILocalizationPipelineStage: IPositionProvider {
public:
    virtual void update() = 0;
    float getX() override;
    float getY() override;
};

class ISpeedController: IDriveSpeedProvider {
public:
    // use updateTarget to do things like generate a new path
    virtual void updateTarget(float positionX, float positionY, float heading) = 0;
    virtual void update(float positionX, float positionY, float heading) = 0;
    float getLeftSpeed() override;
    float getRightSpeed() override;
};

};

#endif