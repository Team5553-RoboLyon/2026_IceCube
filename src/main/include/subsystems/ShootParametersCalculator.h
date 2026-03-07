#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/DriverStation.h"
#include <wpi/interpolating_map.h>
#include "LyonLib/logging/ComplexStructLogger.h"

struct ShootParameters {
    frc::Pose2d lookAheadTargetPos;
    double lookAheadTargetTurretPos = 0.0;
    double hoodAngle = 0.0;
    double flywheelSpeed = 0.0;
};

class ShootParametersCalculator
{
    public:

     ShootParametersCalculator();
     ~ShootParametersCalculator() = default;

     void SetAlliance(frc::DriverStation::Alliance alliance);
     void SetRobotPos(frc::Pose2d robotPos, double timestamp);
     void CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double timestamp);
     void CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double timestamp);


    private:

     frc::Pose2d *m_pLastRobotPos;
     frc::Pose2d *m_pHubTargetPos;
     frc::Pose2d *m_pAllianceZoneTargetPose;
     double m_lastTimestamp = 0.0;
     wpi::interpolating_map<double,double> m_hoodPosMap; //TODO : do tests to get at least 10 values for each interpolating map
     wpi::interpolating_map<double,double> m_flywheelSpeedMap;
     wpi::interpolating_map<double,double> m_timeToReachTargetMap;
     StructLogger<frc::Pose2d> m_logger{"/Hub Pos"};

};