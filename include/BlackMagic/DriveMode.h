#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

namespace BlackMagic {

class DriveMode {
public:
    DriveMode();
    
    virtual void run() = 0;
    virtual bool hasSettled() = 0;
};

class StraightMode: public DriveMode {
public:
    StraightMode();

    void setTarget(float targetInches);
    void run() override;
    bool hasSettled() override;
private:
    float targetInches;
};


class TurnMode: public DriveMode {
public:
    TurnMode();

    void setTarget(float targetHeading);
    void run() override;
    bool hasSettled() override;
private:
    float targetHeading;
};

class ArcMode: public DriveMode {
public:
    ArcMode();
    void run() override;
    bool hasSettled() override;
};

class PipelineMode: public DriveMode {
public:
    PipelineMode();
    void run() override;
    bool hasSettled() override;
};


};

#endif