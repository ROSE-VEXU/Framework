#ifndef DRIVE_STATES_H
#define DRIVE_STATES_H

namespace BlackMagic {

struct Angle {
    enum Unit {
        DEG,
        RAD
    };

    double value;
    Unit units;

    operator float() const {
        return value;
    }

    Angle operator*(int other_value) const {
        return { value*other_value, units };
    }

    Angle asDegrees() const {
        if (units == Unit::DEG) return { value, DEG };
        return { value * 180.0f/M_PI, DEG };
    }

    Angle asRadians() const {
        if (units == Unit::RAD) return { value, RAD };
        return { value * M_PI/180.0f, RAD };
    }
};

struct DrivetrainState {
    float leftDegrees;
    float rightDegrees;
    Angle heading;
};

struct Position {
    float x;
    float y;

    float distanceTo(Position other) {
        float dx = other.x - x;
        float dy = other.y - y;
        return sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    }

    Position inDegrees() {
        return {x * (360.0 / (WHEEL_DIAM_INCHES * M_PI)),
                y * (360.0 / (WHEEL_DIAM_INCHES * M_PI))};
    }
};

struct Pose {
    Position position;
    Angle heading;

    float distanceTo(Pose other) {
        return position.distanceTo(other.position);
    };

    Angle angleTo(Pose other) {
        return { atan2(other.position.y - position.y, other.position.x - position.x), Angle::Unit::RAD };
    }

    Pose rawForTrig() {
        return {position.inDegrees(),
                heading.asRadians()};
    }
};

}; // namespace BlackMagic

constexpr BlackMagic::Angle operator""_deg(long double value) {
    return { static_cast<float>(value), BlackMagic::Angle::Unit::DEG };
}

constexpr BlackMagic::Angle operator""_rad(long double value) {
    return { static_cast<float>(value), BlackMagic::Angle::Unit::RAD };
}


#endif