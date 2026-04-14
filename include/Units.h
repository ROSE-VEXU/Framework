#ifndef UNITS_H
#define UNITS_H

#include "Config.h"
#include "DriveStates.h"
#include <cmath>

constexpr BlackMagic::Angle operator""_deg(long double value) {
    return { static_cast<float>(value), BlackMagic::Angle::Unit::DEG };
}

constexpr BlackMagic::Angle operator""_rad(long double value) {
    return { static_cast<float>(value), BlackMagic::Angle::Unit::RAD };
}

constexpr float operator""_in(long double value) {
    return value * (360.0 / (WHEEL_DIAM_INCHES * M_PI));
}

#endif