#ifndef LEVER_H
#define LEVER_H

class Lever: public BlackMagic::Subsystem {
public:
    Lever(const vex::controller& mainController);
    void opControl() override;
private:
    const vex::controller& mainController;

    enum LeverMode {
        LEVER_OFF,
        LEVER_UP,
        LEVER_DOWN
    };

    LeverMode getLeverMode();
    float getLeverSpeed(LeverMode leverMode);
};

#endif