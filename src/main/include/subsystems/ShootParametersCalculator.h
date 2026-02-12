#include "frc/geometry/Pose2d.h"
#include "frc/DriverStation.h"

struct ShootParameters {
    frc::Pose2d lookAheadTargetPos;
    double lookAheadTargetTurretPos = 0.0;
    double lookAheadTargetDistance = 0.0;
};

class ShootParametersCalculator
{
    public:

     ShootParametersCalculator(frc::Pose2d robotStartingPos);
     ~ShootParametersCalculator() = default;

     void SetAlliance(frc::DriverStation::Alliance alliance);
     void CalculateNewParameters(ShootParameters& params, frc::Pose2d robotPos);

    private:

     frc::Pose2d robotPos;
     bool InBlueAlliance; //if not in blue alliance -> in red alliance
};