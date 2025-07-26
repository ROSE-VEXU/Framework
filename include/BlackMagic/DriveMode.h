#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

namespace BlackMagic {

// template <typename T>
class DriveMode {
public:
    // virtual void setTarget(T target) = 0;
    virtual void run() = 0;
// private:
    // T target;
};

class StraightMode: public DriveMode {
public:
    StraightMode();
    void run() override;
};


class TurnMode: public DriveMode {
public:
    TurnMode();
    void run() override;
};

class ArcMode: public DriveMode {
public:
    ArcMode();
    void run() override;
};

class PipelineMode: public DriveMode {
public:
    PipelineMode();
    void run() override;
};


};

#endif