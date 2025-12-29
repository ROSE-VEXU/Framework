#ifndef LIMITSWITCHAUTOSELECTOR_H
#define LIMITSWITCHAUTOSELECTOR_H

#include "BlackMagic/AutonomousSelector.h"
#include "BlackMagic/AutonomousRoutine.h"

#define AUTO_SELECTOR_NEXT 1
#define AUTO_SELECTOR_PREV -1

class LimitSwitchAutoSelector: public BlackMagic::IAutonomousSelector {
public:
    static LimitSwitchAutoSelector* current_limit_selector_reference;

    LimitSwitchAutoSelector();
    BlackMagic::AutonomousRoutine getSelectedRoutine() override;
    void updateSelectedAuto(int amount);
private:
    int selectedAuto;

    static void nextAuto() {
        if (LimitSwitchAutoSelector::current_limit_selector_reference) {
            LimitSwitchAutoSelector::current_limit_selector_reference->updateSelectedAuto(AUTO_SELECTOR_NEXT);
        }
    }
    static void prevAuto() {
        if (LimitSwitchAutoSelector::current_limit_selector_reference) {
            LimitSwitchAutoSelector::current_limit_selector_reference->updateSelectedAuto(AUTO_SELECTOR_PREV);
        }
    }
    void displaySelectedAuto();
};

#endif