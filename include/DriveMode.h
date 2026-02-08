#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include "AutonomousPipeline.h"
#include "DriveSpeedProvider.h"
#include "PID.h"
#include <functional>
#include <memory>
#include <vector>

namespace BlackMagic {

class IDriveMode: IDriveSpeedProvider {
public:
    virtual void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) = 0;
    virtual bool hasSettled(const DrivetrainState& drive_state) = 0;
    DriveSpeeds getSpeeds() = 0;

protected:
    int settle_count;
};

class StraightMode: public IDriveMode {
public:
    StraightMode();

    void setTarget(float target_inches, Angle target_heading);
    void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_deg;
    Angle target_heading;
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

    void setTarget(Angle target_heading);
    void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) override;
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

struct ArcSettings {
    float start_arc_pct;
    float arc_length_pct;
};

class ArcMode: public IDriveMode {
public:
    ArcMode();

    void setTarget(float target_inches, Angle target_heading, ArcSettings arc_settings);
    void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_deg;
    Angle target_heading;
    ArcSettings arc_settings;
    float linear_speed;
    float angular_speed;
    float settling_prev_left;
    float settling_prev_right;
    float settling_total_left;
    float settling_total_right;
};

struct CurveKeyframe {
    float pct;
    Angle heading;
    float smooth_pct;
};

class CurveMode: public IDriveMode {
public:
    CurveMode();

    void setTarget(float target_inches, std::vector<CurveKeyframe> keyframes);
    void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) override;
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;
private:
    float target_deg;
    std::vector<CurveKeyframe> keyframes;
    int curr_keyframe_index;
    float linear_speed;
    float angular_speed;
    float settling_prev_left;
    float settling_prev_right;
    float settling_total_left;
    float settling_total_right;
};

class PipelineMode: public IDriveMode {
public:
    PipelineMode();

    void setTarget(Pose target_pose);
    void run(const DrivetrainState& drive_state, PID& linear_pid, PID& angular_pid) override;
    void setPipeline(std::shared_ptr<AutonomousPipeline> pipeline);
    bool hasSettled(const DrivetrainState& drive_state) override;
    DriveSpeeds getSpeeds() override;

private:
    std::shared_ptr<AutonomousPipeline> pipeline;
};


};

#endif