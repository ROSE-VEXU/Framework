#ifndef AUTONOMOUS_PIPELINE_H
#define AUTONOMOUS_PIPELINE_H

#include "PID.h"
#include "AutonomousPipelineStages.h"
#include "PositionProvider.h"
#include <memory>

namespace BlackMagic {

class AutonomousPipeline: IPositionProvider, IDriveSpeedProvider {
public:
    AutonomousPipeline();

    template<typename OdometryImplementationType>
    AutonomousPipeline&& withOdometrySource(OdometryImplementationType&& odometrySource) && {
        VERIFY_SUBCLASS(std::decay_t<OdometryImplementationType>, IOdometryPipelineStage, "withOdometrySource", "odometrySource", "IOdometryPipelineStage");
        this->odometrySource = std::make_unique<std::decay_t<OdometryImplementationType>>(std::forward<OdometryImplementationType>(odometrySource));
        return std::move(*this);
    }
    template<typename OdometryImplementationType>
    AutonomousPipeline& withOdometrySource(OdometryImplementationType&& odometrySource) & {
        VERIFY_SUBCLASS(std::decay_t<OdometryImplementationType>, IOdometryPipelineStage, "withOdometrySource", "odometrySource", "IOdometryPipelineStage");
        this->odometrySource = std::make_unique<std::decay_t<OdometryImplementationType>>(std::forward<OdometryImplementationType>(odometrySource));
        return *this;
    }

    template<typename LocalizationImplementationType>
    AutonomousPipeline&& withLocalizationSource(LocalizationImplementationType&& localizationSource) && {
        VERIFY_SUBCLASS(std::decay_t<LocalizationImplementationType>, ILocalizationPipelineStage, "withLocalizationSource", "localizationSource", "ILocalizationPipelineStage");
        this->localizationSource = std::make_unique<std::decay_t<LocalizationImplementationType>>(std::forward<LocalizationImplementationType>(localizationSource));
        return std::move(*this);
    }
    template<typename LocalizationImplementationType>
    AutonomousPipeline& withLocalizationSource(LocalizationImplementationType&& localizationSource) & {
        VERIFY_SUBCLASS(std::decay_t<LocalizationImplementationType>, ILocalizationPipelineStage, "withLocalizationSource", "localizationSource", "ILocalizationPipelineStage");
        this->localizationSource = std::make_unique<std::decay_t<LocalizationImplementationType>>(std::forward<LocalizationImplementationType>(localizationSource));
        return *this;
    }

    template<typename SpeedControllerType>
    AutonomousPipeline&& withSpeedController(SpeedControllerType&& speedController) && {
        VERIFY_SUBCLASS(std::decay_t<SpeedControllerType>, ISpeedController, "withSpeedController", "speedController", "ISpeedController");
        this->speedController = std::make_unique<std::decay_t<SpeedControllerType>>(std::forward<SpeedControllerType>(speedController));
        return std::move(*this);
    }
    template<typename SpeedControllerType>
    AutonomousPipeline& withSpeedController(SpeedControllerType&& speedController) & {
        VERIFY_SUBCLASS(std::decay_t<SpeedControllerType>, ISpeedController, "withSpeedController", "speedController", "ISpeedController");
        this->speedController = std::make_unique<std::decay_t<SpeedControllerType>>(std::forward<SpeedControllerType>(speedController));
        return *this;
    }

    void setTarget(Pose target_pose);
    int runPipeline(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid);
    bool hasSettled(const DriveModeUtilFunctions& utils);

    Position getPosition() override;
    DriveSpeeds getSpeeds() override;

private:
    std::unique_ptr<IOdometryPipelineStage> odometrySource;
    std::unique_ptr<ILocalizationPipelineStage> localizationSource;
    std::unique_ptr<ISpeedController> speedController;

    Pose target_pose;

    Position odomPosition;
    Position localizedPosition;
    DriveSpeeds outputSpeeds;
};

};

#endif