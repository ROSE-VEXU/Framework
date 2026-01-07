#include "vex.h"

namespace BlackMagic {

DriveControllerMovement::DriveControllerMovement(const vex::controller::axis& first_axis, const vex::controller::axis& second_axis): first_axis(first_axis), second_axis(second_axis) {
}

};