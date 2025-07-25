#ifndef POSITION_PROVIDER_H
#define POSITION_PROVIDER_H

namespace BlackMagic {

class PositionProvider {
public:
    virtual float getX() = 0;
    virtual float getY() = 0;
};

};

#endif