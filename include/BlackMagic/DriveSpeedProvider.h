#ifndef DRIVE_SPEED_PROVIDER_H
#define DRIVE_SPEED_PROVIDER_H

namespace BlackMagic {

class IDriveSpeedProvider {
public:
    virtual float getLeftSpeed() = 0;
    virtual float getRightSpeed() = 0;
};

};

#endif