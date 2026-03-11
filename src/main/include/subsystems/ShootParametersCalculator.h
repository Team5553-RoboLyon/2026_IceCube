#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/DriverStation.h"
#include <wpi/interpolating_map.h>
#include "LyonLib/logging/ComplexStructLogger.h"
#include "LyonLib/utils/MacroUtilsRBL.h"

struct ShootParameters {
    frc::Pose2d lookAheadTargetPos;
    double lookAheadTargetTurretPos = 0.0;
    double hoodAngle = 0.0;
    double flywheelSpeed = 0.0;
    bool isTargetInDeadZone = true;
    bool turretInTolerance;
};

struct vec2D
{
    double x;
    double y;
};

class ShootParametersCalculator
{
    public:

     ShootParametersCalculator();
     ~ShootParametersCalculator() = default;

     void SetAlliance(frc::DriverStation::Alliance alliance);
     void SetTurretPos(frc::Pose2d robotPos, double turretOrientation, double timestamp);
     void CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp);
     void CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp);

     double PIPI(double angle_rad);

    private:

     frc::Pose2d m_lastTurretPos;
     frc::Pose2d m_hubTargetPos;
     frc::Pose2d m_allianceZoneTargetPose;
     double m_lastTimestamp = 0.0;
     double m_turretTolerance = NF64_PI/6.0;
     wpi::interpolating_map<double,double> m_hoodPosMap; //TODO : do tests to get at least 10 values for each interpolating map
     wpi::interpolating_map<double,double> m_flywheelSpeedMap;
     wpi::interpolating_map<double,double> m_timeToReachTargetMap;
     StructLogger<frc::Pose2d> m_logger{"/HubPos"};
     StructLogger<frc::Pose2d> m_projectedLogger{"/ProjectedTarget"};
     StructLogger<frc::Pose2d> m_turretLogger{"/TurretPos"};

};