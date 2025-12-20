#include "./BlackMagic/PipelineStages.h"
#include "./BlackMagic/PositionProvider.h"
#include "./BlackMagic/DriveSpeedProvider.h"

class DriveToPoseSpeedController: public BlackMagic::ISpeedController {
public:
    DriveToPoseSpeedController();
    void updateTarget(BlackMagic::Position targetPosition, float heading) override;
    void update(BlackMagic::Position currentPosition, float heading) override;
    BlackMagic::DriveSpeeds getSpeeds() override;
private:
    BlackMagic::Position targetPosition;
    float targetHeading;

    float leftSpeed;
    float rightSpeed;

    BlackMagic::DriveSpeeds getScaledSpeedsFromMax(float linearSpeed, float angularSpeed);
};