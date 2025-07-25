#include "vex.h"

namespace BlackMagic {

AutonomousPipeline::AutonomousPipeline(): odometrySource(),
                                          localizationSource(),
                                          speedController() {}

AutonomousPipeline& AutonomousPipeline::withOdometrySource(OdometryPipelineStage odometrySource) {
    this->odometrySource = std::move(odometrySource);
    return *this;
}

AutonomousPipeline& AutonomousPipeline::withLocalizationSource(LocalizationPipelineStage localizationSource) {
    this->localizationSource = std::move(localizationSource);
    return *this;
}

AutonomousPipeline& AutonomousPipeline::withSpeedController(SpeedController speedController) {
    this->speedController = std::move(speedController);
    return *this;
}


};
