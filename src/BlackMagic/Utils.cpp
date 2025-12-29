#include "vex.h"

namespace BlackMagic {

float Utils::getShortestAngleBetween(float current_heading, float target_heading) {
    return fmod((target_heading-current_heading+540.0), 360.0)-180.0;
}

int Utils::sign(int value) {
    return value < 0 ? -1 : 1;
}

int Utils::sign(float value) {
    return value < 0.0 ? -1 : 1;
}

};