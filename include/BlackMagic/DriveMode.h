#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

namespace BlackMagic {

// template <typename T>
class DriveMode {
public:
    DriveMode();
    // virtual void setTarget(T target) = 0;
    virtual void run() = 0;
    virtual void hasSettled() = 0;
// private:
    // T target;
};

class StraightMode: public DriveMode {
public:
    StraightMode();
    void run() override;
    void hasSettled() override;
};


class TurnMode: public DriveMode {
public:
    TurnMode();
    void run() override;
    void hasSettled() override;
};

class ArcMode: public DriveMode {
public:
    ArcMode();
    void run() override;
    void hasSettled() override;
};

class PipelineMode: public DriveMode {
public:
    PipelineMode();
    void run() override;
    void hasSettled() override;
};


};

#endif