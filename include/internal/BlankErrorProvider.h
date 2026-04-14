#ifndef BLANK_ERROR_PROVIDER_H
#define BLANK_ERROR_PROVIDER_H

#include "../ErrorProvider.h"

namespace BlackMagic {

// Used only as a placeholder for ZERO_PID
class BlankErrorProvider: public IErrorProvider {
protected:
    BlankErrorProvider();
    float getError(float target) override;
};

};

#endif