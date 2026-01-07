#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include "AutonomousPipeline.h"
#include "DriveSpeedProvider.h"
#include "PID.h"
#include <functional>
#include <memory>

namespace BlackMagic {

class IDriveMode: IDriveSpeedProvider {
public:
    virtual void run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) = 0;
    virtual bool hasSettled(const DrivetrainState& drive_state) = 0;
    DriveSpeeds getSpeeds() = 0;

protected:
    int settle_count;
};

class StraightMode: public IDriveMode {
public:
    StraightMode() = default;

    void setTarget(float target_inches, Angle target_heading);
    void run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_deg;
    Angle target_heading;
    bool decelerating;
    float linear_speed;
    float angular_speed;
    int settle_count;
    float settling_prev_left;
    float settling_prev_right;
    float settling_total_left;
    float settling_total_right;
};

class TurnMode: public IDriveMode {
public:
    TurnMode() = default;

    void setTarget(Angle targetHeading);
    void run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    Angle target_heading;
    bool decelerating;
    float left_speed;
    float right_speed;
    float settling_prev_heading;
    float settling_total_heading_change;
};

class ArcMode: public IDriveMode {
public:
    ArcMode() = default;

    void run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
};

class PipelineMode: public IDriveMode {
public:
    PipelineMode() = default;

    void setTarget(float targetX, float targetY, float targetHeading);
    void run(const DrivetrainState& drive_state, std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    void setPipeline(std::shared_ptr<AutonomousPipeline> pipeline);
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;

private:
    std::shared_ptr<AutonomousPipeline> pipeline;
};


};

#endif