#ifndef ERROR_PROVIDER_H
#define ERROR_PROVIDER_H

#include <type_traits>
namespace BlackMagic {

template<typename ErrorType>
class IErrorProvider {
    static_assert(std::is_convertible<ErrorType, float>::value, "ErrorType must be convertible to float for PID error calculations.");

public:
    virtual ErrorType getError(ErrorType target) = 0;
};

};

#endif