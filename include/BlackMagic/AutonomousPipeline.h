#ifndef AUTONOMOUS_PIPELINE_H
#define AUTONOMOUS_PIPELINE_H

#include "PipelineStages.h"
#include <memory>

namespace BlackMagic {

class AutonomousPipeline {
public:
    AutonomousPipeline();
    AutonomousPipeline& withOdometrySource(OdometryPipelineStage odometrySource);
    AutonomousPipeline& withLocalizationSource(LocalizationPipelineStage localizationSource);
    AutonomousPipeline& withSpeedController(SpeedController speedController);
private:
    OdometryPipelineStage odometrySource;
    LocalizationPipelineStage localizationSource;
    SpeedController speedController;
};

};

#endif