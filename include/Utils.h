#ifndef UTILS_H
#define UTILS_H

#include "DriveStates.h"

namespace BlackMagic {

class Utils {
public:
    static vex::brain& robot_brain();
    static Angle getShortestAngleBetween(Angle current_heading, Angle target_heading);
    static Angle getAngleError(Angle target_heading, Angle current_heading);
    static int sign(int value);
    static int sign(float value);
    static float clamp(float value, float min, float max);
    static DriveSpeeds desaturateSpeeds(float linear_speed, float angular_speed);
};

};

#endif