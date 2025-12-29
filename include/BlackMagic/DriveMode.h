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
    virtual bool hasSettled(const DriveModeUtilFunctions& utils) = 0;
    DriveSpeeds getSpeeds() = 0;

protected:
    int settle_count;
    int max_settle_count = 6;
};

class StraightMode: public IDriveMode {
public:
    StraightMode() = default;

    void setTarget(float targetInches);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DriveModeUtilFunctions& utils) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_deg;
    float speed;
    float settling_prev_position;
    float settling_total_position_change;
    float settling_position_threshold = 5.0;
};

class TurnMode: public IDriveMode {
public:
    TurnMode() = default;

    void setTarget(float targetHeading);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DriveModeUtilFunctions& utils) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_heading;
    float left_speed;
    float right_speed;
    float settling_prev_heading;
    float settling_total_heading_change;
    float settling_angle_threshold = 0.35;
};

class ArcMode: public IDriveMode {
public:
    ArcMode() = default;

    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DriveModeUtilFunctions& utils) override;
    DriveSpeeds getSpeeds() override;
};

class PipelineMode: public IDriveMode {
public:
    PipelineMode() = default;

    void setTarget(float targetX, float targetY, float targetHeading);
    void run(const DriveModeUtilFunctions& utils, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    void setPipeline(std::shared_ptr<AutonomousPipeline> pipeline);
    bool hasSettled(const DriveModeUtilFunctions& utils) override;
    DriveSpeeds getSpeeds() override;

private:
    std::shared_ptr<AutonomousPipeline> pipeline;
};


};

#endif