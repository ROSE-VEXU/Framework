#ifndef POSITION_PROVIDER_H
#define POSITION_PROVIDER_H

#include "DriveStates.h"

namespace BlackMagic {

class IPositionProvider {
public:
    virtual Position getPosition() = 0;
};

};

#endif