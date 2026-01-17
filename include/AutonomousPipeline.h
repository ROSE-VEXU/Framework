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
        this->odometry_source = std::make_unique<std::decay_t<OdometryImplementationType>>(std::forward<OdometryImplementationType>(odometrySource));
        return std::move(*this);
    }
    template<typename OdometryImplementationType>
    AutonomousPipeline& withOdometrySource(OdometryImplementationType&& odometrySource) & {
        VERIFY_SUBCLASS(std::decay_t<OdometryImplementationType>, IOdometryPipelineStage, "withOdometrySource", "odometrySource", "IOdometryPipelineStage");
        this->odometry_source = std::make_unique<std::decay_t<OdometryImplementationType>>(std::forward<OdometryImplementationType>(odometrySource));
        return *this;
    }

    template<typename LocalizationImplementationType>
    AutonomousPipeline&& withLocalizationSource(LocalizationImplementationType&& localizationSource) && {
        VERIFY_SUBCLASS(std::decay_t<LocalizationImplementationType>, ILocalizationPipelineStage, "withLocalizationSource", "localizationSource", "ILocalizationPipelineStage");
        this->localization_source = std::make_unique<std::decay_t<LocalizationImplementationType>>(std::forward<LocalizationImplementationType>(localizationSource));
        return std::move(*this);
    }
    template<typename LocalizationImplementationType>
    AutonomousPipeline& withLocalizationSource(LocalizationImplementationType&& localizationSource) & {
        VERIFY_SUBCLASS(std::decay_t<LocalizationImplementationType>, ILocalizationPipelineStage, "withLocalizationSource", "localizationSource", "ILocalizationPipelineStage");
        this->localization_source = std::make_unique<std::decay_t<LocalizationImplementationType>>(std::forward<LocalizationImplementationType>(localizationSource));
        return *this;
    }

    template<typename SpeedControllerType>
    AutonomousPipeline&& withSpeedController(SpeedControllerType&& speedController) && {
        VERIFY_SUBCLASS(std::decay_t<SpeedControllerType>, ISpeedController, "withSpeedController", "speedController", "ISpeedController");
        this->speed_controller = std::make_unique<std::decay_t<SpeedControllerType>>(std::forward<SpeedControllerType>(speedController));
        return std::move(*this);
    }
    template<typename SpeedControllerType>
    AutonomousPipeline& withSpeedController(SpeedControllerType&& speedController) & {
        VERIFY_SUBCLASS(std::decay_t<SpeedControllerType>, ISpeedController, "withSpeedController", "speedController", "ISpeedController");
        this->speed_controller = std::make_unique<std::decay_t<SpeedControllerType>>(std::forward<SpeedControllerType>(speedController));
        return *this;
    }

    void setTarget(Pose target_pose);
    void setPosition(Position position);
    int runPipeline(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid);
    bool hasSettled(const DrivetrainState& drive_state);

    Position getPosition() override;
    DriveSpeeds getSpeeds() override;

private:
    std::unique_ptr<IOdometryPipelineStage> odometry_source;
    std::unique_ptr<ILocalizationPipelineStage> localization_source;
    std::unique_ptr<ISpeedController> speed_controller;

    Pose target_pose;

    Position odom_position;
    Position localized_position;
};

};

#endif