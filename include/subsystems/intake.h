#ifndef INTAKE_H
#define INTAKE_H

class Intake: public BlackMagic::Subsystem {
public:
    Intake(const vex::controller::button& in_button, const vex::controller::button& out_button);
    void opControl() override;
private:
    const vex::controller::button& in_button;
    const vex::controller::button& out_button;
    
    enum IntakeMode {
        INTAKE_OFF,
        INTAKE_IN,
        INTAKE_OUT        
    };

    IntakeMode getIntakeMode();
    float getIntakeSpeed(IntakeMode intakeMode);
};

#endif