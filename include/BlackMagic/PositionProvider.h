#ifndef POSITION_PROVIDER_H
#define POSITION_PROVIDER_H

namespace BlackMagic {

struct Position {
    float x;
    float y;

    float distanceTo(Position other) {
        float dx = other.x - x;
        float dy = other.y - y;
        return sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    }
};

class IPositionProvider {
public:
    virtual Position getPosition() = 0;
};

};

#endif