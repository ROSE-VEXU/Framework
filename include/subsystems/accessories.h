#ifndef ACCESSORIES_H
#define ACCESSORIES_H

class Accessories: public BlackMagic::Subsystem {
public:
    Accessories(const vex::controller::button& lever_button, const vex::controller::button& little_will_button);
    void opControl() override;
private:
    const vex::controller::button& lever_button;
    const vex::controller::button& little_will_button;
};

#endif