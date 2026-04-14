#ifndef ERROR_PROVIDER_H
#define ERROR_PROVIDER_H

#include <type_traits>
namespace BlackMagic {

class IErrorProvider {
public:
    virtual float getError(float target) = 0;
    virtual float getRawValue() = 0;
};

};

#endif