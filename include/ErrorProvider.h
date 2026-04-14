#ifndef ERROR_PROVIDER_H
#define ERROR_PROVIDER_H

#include <type_traits>
namespace BlackMagic {

struct SettleConfig {
    int max_settle_count;
    float reset_settle_threshold;
};

class IErrorProvider {
public:
    IErrorProvider(SettleConfig settle_config): settle_config(settle_config), previous_raw_value(0) {}
    virtual float getError(float target) = 0;
    virtual float getRawValue() = 0;
    virtual bool hasSettled() = 0;
private:
    SettleConfig settle_config;
    float previous_raw_value;
};

};

#endif