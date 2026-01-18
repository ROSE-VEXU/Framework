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
    void setMaxSpeed(float max_speed) {
        this->max_speed = max_speed;
    }
    virtual void run(const DrivetrainState& drive_state) = 0;
    virtual bool hasSettled(const DrivetrainState& drive_state) = 0;
    DriveSpeeds getSpeeds() = 0;

protected:
    int settle_count;
    float max_speed;
};

class StraightMode: public IDriveMode {
public:
    StraightMode();

    void setPIDs(PID& linear_pid, PID& angular_pid);
    void setTarget(float target_inches, Angle target_heading);
    void run(const DrivetrainState& drive_state) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    PID& linear_pid;
    PID& angular_pid;
    float max_speed;
    
    float target_deg;
    Angle target_heading;
    bool decelerating;
    float linear_speed;
    float angular_speed;
    float settling_prev_left;
    float settling_prev_right;
    float settling_total_left;
    float settling_total_right;
};

class TurnMode: public IDriveMode {
public:
    TurnMode();

    void setPIDs(PID& angular_pid);
    void setTarget(Angle target_heading);
    void run(const DrivetrainState& drive_state) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    PID& angular_pid;
    float max_speed;

    Angle target_heading;
    bool decelerating;
    float left_speed;
    float right_speed;
    float settling_prev_heading;
    float settling_total_heading_change;
};

class PipelineMode: public IDriveMode {
public:
    PipelineMode();

    void setPIDs(PID& linear_pid, PID& angular_pid);
    void setTarget(Pose target_pose);
    void run(const DrivetrainState& drive_state) override;
    void setPipeline(std::shared_ptr<AutonomousPipeline> pipeline);
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;

private:
    PID& linear_pid;
    PID& angular_pid;
    float max_speed;

    std::shared_ptr<AutonomousPipeline> pipeline;
};


};

#endif