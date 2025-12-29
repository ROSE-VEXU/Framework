#include "vex.h"

vex::motor leftIntake = vex::motor(vex::PORT4, true);
vex::motor rightIntake = vex::motor(vex::PORT6);

Intake::Intake(const vex::controller::button& in_button, const vex::controller::button& out_button) : 
    in_button(in_button), out_button(out_button) {
}

void Intake::opControl() {
    float speed = getIntakeSpeed(getIntakeMode());
    leftIntake.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    rightIntake.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}

Intake::IntakeMode Intake::getIntakeMode() {
    if (in_button.pressing()) {
        return INTAKE_IN;
    } else if (out_button.pressing()) {
        return INTAKE_OUT;
    }

    return INTAKE_OFF;
}

float Intake::getIntakeSpeed(Intake::IntakeMode intakeMode) {
    switch (intakeMode) {
        case INTAKE_OFF:
            return 0.0;
        case INTAKE_IN:
            return 100.0;
        case INTAKE_OUT:
            return -100.0;
        default:
            return 0.0;
    }
}