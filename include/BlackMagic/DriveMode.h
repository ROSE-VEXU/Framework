#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include "AutonomousPipeline.h"
#include "DriveSpeedProvider.h"
#include "PID.h"
#include <functional>
#include <memory>

namespace BlackMagic {

struct DriveModeUtilFunctions {
    const std::function<float()>& getLeftDegrees;
    const std::function<float()>& getRightDegrees;
    const std::function<float()>& getHeading;
};

class IDriveMode: IDriveSpeedProvider {
public:
    virtual void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) = 0;
    virtual bool hasSettled() = 0;
    DriveSpeeds getSpeeds() = 0;
};

class StraightMode: public IDriveMode {
public:
    StraightMode() = default;

    void setTarget(float targetInches);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
    DriveSpeeds getSpeeds() override;
private:
    float targetInches;
};

class TurnMode: public IDriveMode {
public:
    TurnMode() = default;

    void setTarget(float targetHeading);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
    DriveSpeeds getSpeeds() override;
private:
    float targetHeading;
};

class ArcMode: public IDriveMode {
public:
    ArcMode() = default;

    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
    DriveSpeeds getSpeeds() override;
};

class PipelineMode: public IDriveMode {
public:
    PipelineMode() = default;

    void setTarget(float targetX, float targetY, float targetHeading);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    void setPipeline(std::shared_ptr<AutonomousPipeline> pipeline);
    bool hasSettled() override;
    DriveSpeeds getSpeeds() override;

private:
    std::shared_ptr<AutonomousPipeline> pipeline;
};


};

#endif