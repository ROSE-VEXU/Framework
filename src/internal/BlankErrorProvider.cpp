#include "internal/BlankErrorProvider.h"

namespace BlackMagic {

BlankErrorProvider::BlankErrorProvider() {}

float BlankErrorProvider::getError(float target) {
    return 0.0;
}

}