#include "vex.h"

namespace BlackMagic {

AutonomousSelector::AutonomousSelector(): routines({}) {}

AutonomousRoutine AutonomousSelector::getSelectedRoutine() {
    return routines[0]; // TODO - Implement selection logic
}

void AutonomousSelector::addRoutine(AutonomousRoutine routine) {
    routines.push_back(routine);
}
    
};
