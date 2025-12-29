#ifndef UTILS_H
#define UTILS_H

namespace BlackMagic {

class Utils {
public:
    static float getShortestAngleBetween(float current_heading, float target_heading);
    static int sign(int value);
    static int sign(float value);
};

};

#endif