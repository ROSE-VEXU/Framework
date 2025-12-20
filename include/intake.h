#ifndef INTAKE_H
#define INTAKE_H

class Intake: public BlackMagic::Subsystem {
public:
    Intake(const vex::controller& mainController);
    void opControl() override;
private:
    const vex::controller& mainController;

    enum IntakeMode {
        INTAKE_OFF,
        INTAKE_IN,
        INTAKE_OUT        
    };

    IntakeMode getIntakeMode();
    float getIntakeSpeed(IntakeMode intakeMode);
};

#endif