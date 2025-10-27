#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include "AutonomousPipeline.h"
#include "PID.h"
#include <functional>
#include <memory>

namespace BlackMagic {

struct DriveModeUtilFunctions {
    const std::function<float()>& getLeftDegrees;
    const std::function<float()>& getRightDegrees;
    const std::function<float()>& getHeading;
};

class DriveMode {
public:
    DriveMode(const DriveModeUtilFunctions& utils);

    virtual void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) = 0;
    virtual bool hasSettled() = 0;
protected:
    DriveModeUtilFunctions utils;
};

class StraightMode: public DriveMode {
public:
    StraightMode(const DriveModeUtilFunctions& utils);

    void setTarget(float targetInches);
    void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
private:
    float targetInches;
};

class TurnMode: public DriveMode {
public:
    TurnMode(const DriveModeUtilFunctions& utils);

    void setTarget(float targetHeading);
    void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
private:
    float targetHeading;
};

class ArcMode: public DriveMode {
public:
    ArcMode(const DriveModeUtilFunctions& utils);

    void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    bool hasSettled() override;
};

class PipelineMode: public DriveMode {
public:
    PipelineMode(const DriveModeUtilFunctions& utils);

    void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid) override;
    void run(std::shared_ptr<PID> linear_pid, std::shared_ptr<PID> angular_pid, AutonomousPipeline pipeline);
    bool hasSettled() override;
};


};

#endif