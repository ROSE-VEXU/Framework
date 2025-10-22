#include "./BlackMagic/PipelineStages.h"

class DriveToPoseSpeedController: public BlackMagic::ISpeedController {
public:
    DriveToPoseSpeedController();
    void updateTarget(float positionX, float positionY, float heading) override;
    void update(float positionX, float positionY, float heading) override;
    float getLeftSpeed() override;
    float getRightSpeed() override;
private:
    float targetPositionX;
    float targetPositionY;
    float targetHeading;

    float leftRawSpeed;
    float rightRawSpeed;
};