#include "internal/BlankErrorProvider.h"

namespace BlackMagic {

BlankErrorProvider::BlankErrorProvider() {}

float BlankErrorProvider::getError(float target) {
    return 0.0;
}

bool BlankErrorProvider::hasSettled() {
    return true;
}

}