#ifndef HEADING_PROVIDER_H
#define HEADING_PROVIDER_H

#include "DriveStates.h"

namespace BlackMagic {

class IHeadingProvider {
public:
    virtual void calibrate() = 0;
    virtual Angle getHeading() = 0;
};

};

#endif