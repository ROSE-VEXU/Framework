#include "internal/BlankErrorProvider.h"

namespace BlackMagic {

BlankErrorProvider::BlankErrorProvider(): IErrorProvider({ 0, 0.0 }) {}

float BlankErrorProvider::getError(float target) {
    return 0.0;
}

bool BlankErrorProvider::hasSettled() {
    return true;
}

}