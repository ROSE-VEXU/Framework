#ifndef PIPELINE_STAGES_H
#define PIPELINE_STAGES_H

#include "DriveSpeedProvider.h"
#include "PositionProvider.h"

namespace BlackMagic {

class OdometryPipelineStage: PositionProvider {
public:
    OdometryPipelineStage();
    void update();
    float getX() override;
    float getY() override;
};

class LocalizationPipelineStage: PositionProvider {
public:
    LocalizationPipelineStage();
    void update();
    float getX() override;
    float getY() override;
};

class SpeedController: DriveSpeedProvider {
public:
    SpeedController();
    void update();
    float getLeftSpeed() override;
    float getRightSpeed() override;
};

};

#endif