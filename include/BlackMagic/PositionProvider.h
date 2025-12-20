#ifndef POSITION_PROVIDER_H
#define POSITION_PROVIDER_H

namespace BlackMagic {

struct Position {
    float x;
    float y;
};

class IPositionProvider {
public:
    virtual Position getPosition() = 0;
};

};

#endif