#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include "PID.h"
#include <memory>

namespace BlackMagic {

class DriveMode {
public:
    DriveMode();

    virtual void run(std::shared_ptr<PID> pid) = 0;
    virtual bool hasSettled() = 0;
};

class StraightMode: public DriveMode {
public:
    StraightMode();

    void setTarget(float targetInches);
    void run(std::shared_ptr<PID> pid) override;
    bool hasSettled() override;
private:
    float targetInches;
};


class TurnMode: public DriveMode {
public:
    TurnMode();

    void setTarget(float targetHeading);
    void run(std::shared_ptr<PID> pid) override;
    bool hasSettled() override;
private:
    float targetHeading;
};

class ArcMode: public DriveMode {
public:
    ArcMode();

    void run(std::shared_ptr<PID> pid) override;
    bool hasSettled() override;
};

class PipelineMode: public DriveMode {
public:
    PipelineMode();

    void run(std::shared_ptr<PID> pid) override;
    bool hasSettled() override;
};


};

#endif