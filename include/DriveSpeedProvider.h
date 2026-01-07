#ifndef DRIVE_SPEED_PROVIDER_H
#define DRIVE_SPEED_PROVIDER_H

namespace BlackMagic {

struct DriveSpeeds {
    float left;
    float right;
};

class IDriveSpeedProvider {
public:
    virtual DriveSpeeds getSpeeds() = 0;
};

};

#endif