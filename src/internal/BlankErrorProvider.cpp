#include "internal/BlankErrorProvider.h"

namespace BlackMagic {

BlackMagic::BlankErrorProvider() {}

float BlankErrorProvider::getError(float target) {
    return 0.0;
}

}