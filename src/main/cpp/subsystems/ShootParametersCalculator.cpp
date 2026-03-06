#include "subsystems/ShootParametersCalculator.h"

#include "frc/DriverStation.h"
#include "subsystems/turret/TurretConstants.h"
#include "subsystems/shooter/hood/HoodConstants.h"
#include "subsystems/shooter/flywheel/FlywheelConstants.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

ShootParametersCalculator::ShootParametersCalculator()
{
    m_hoodPosMap.insert(0.51, 4.5);
    m_hoodPosMap.insert(1.01, 7);
    m_hoodPosMap.insert(1.51, 8.5);

    m_flywheelSpeedMap.insert(0.51, 2300.0);
    m_flywheelSpeedMap.insert(1.01, 2450.0);
    m_flywheelSpeedMap.insert(1.51, 2550.0);

    m_timeToReachTargetMap.insert(0.51, 0.6);
    m_timeToReachTargetMap.insert(1.01, 0.8);
    m_timeToReachTargetMap.insert(1.51, 1);
}

void ShootParametersCalculator::SetAlliance(frc::DriverStation::Alliance alliance)
{
    if (alliance == frc::DriverStation::Alliance::kBlue)
    {
        m_hubTargetPos = FieldConstants::Hub::BLUE_PLACEMENT;
        m_allianceZoneTargetPose = FieldConstants::AllianceZone::BLUE_CENTER_POSITION;
    }
    else
    {
        m_hubTargetPos = FieldConstants::Hub::RED_PLACEMENT;
        m_allianceZoneTargetPose = FieldConstants::AllianceZone::RED_CENTER_POSITION;
    }

}

void ShootParametersCalculator::SetRobotPos(frc::Pose2d robotPos, double timestamp)
{
    m_lastRobotPos = robotPos;
    m_lastTimestamp = timestamp;
}

void ShootParametersCalculator::CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double timestamp)
{ 
    frc::Transform2d robotDisplacement = robotPos - m_lastRobotPos;
    double elapsedTime = m_lastTimestamp-timestamp;

    double xSpeed = robotDisplacement.X().value()/(elapsedTime);
    double ySpeed = robotDisplacement.Y().value()/(elapsedTime);
    double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(xSpeed * TIME_PER_CYCLE), 
                                                                  units::meter_t(ySpeed * TIME_PER_CYCLE),
                                                                  units::radian_t(rotationSpeed * TIME_PER_CYCLE)});
    frc::Pose2d estimatedNextTurretPos = estimatedNextRobotPos + TurretConstants::Specifications::ROBOT_TO_TURRET;

    double nextTurretToHubDistance = (m_hubTargetPos - estimatedNextTurretPos).Translation().Norm().value();
    double timeToReachHub = m_timeToReachTargetMap[nextTurretToHubDistance];

    frc::Pose2d lookAheadTargetPos = m_hubTargetPos;
    double lookAheadTargetDistance = nextTurretToHubDistance;
    double timeToReachLookAheadTarget = timeToReachHub;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeToReachLookAheadTarget = m_timeToReachTargetMap[lookAheadTargetDistance];
        lookAheadTargetPos = m_hubTargetPos + frc::Transform2d {units::meter_t(-xSpeed*timeToReachLookAheadTarget), 
                                                           units::meter_t(-ySpeed*timeToReachLookAheadTarget),
                                                           0.0_rad};
        lookAheadTargetDistance = (lookAheadTargetPos - estimatedNextTurretPos).Translation().Norm().value();
    }

    params.hoodAngle = m_hoodPosMap[lookAheadTargetDistance];
    if (params.hoodAngle < HoodConstants::Position::MIN)
    {
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_HUB;
        params.hoodAngle = HoodConstants::Position::MIN;
    }
    else if (params.hoodAngle > HoodConstants::Position::MAX)
    {
        params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];

    double turretSuplAngleToAlignToTarget =  atan((lookAheadTargetPos-estimatedNextTurretPos).Y().value()/(lookAheadTargetPos-estimatedNextTurretPos).X().value()); //TODO : find a better name

    params.lookAheadTargetTurretPos = WRAP_ANGLE_NEG_PI_TO_PI(m_hubTargetPos.Rotation().Radians().value() - estimatedNextTurretPos.Rotation().Radians().value()
                                      + turretSuplAngleToAlignToTarget);

    params.lookAheadTargetPos = lookAheadTargetPos;

    m_lastRobotPos = robotPos;
    m_lastTimestamp = timestamp;
}

void ShootParametersCalculator::CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double timestamp)
{
    frc::Transform2d robotDisplacement = robotPos - m_lastRobotPos;
    double elapsedTime = m_lastTimestamp-timestamp;

    double xSpeed = robotDisplacement.X().value()/(elapsedTime);
    double ySpeed = robotDisplacement.Y().value()/(elapsedTime);
    double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(xSpeed * TIME_PER_CYCLE), 
                                                                  units::meter_t(ySpeed * TIME_PER_CYCLE),
                                                                  units::radian_t(rotationSpeed * TIME_PER_CYCLE)});
    frc::Pose2d estimatedNextTurretPos = estimatedNextRobotPos + TurretConstants::Specifications::ROBOT_TO_TURRET;

    double nextTurretToAllianceZoneDistance = (m_allianceZoneTargetPose - estimatedNextTurretPos).Translation().Y().value();
    double timeToReachAllianceZone = m_timeToReachTargetMap[nextTurretToAllianceZoneDistance];

    frc::Pose2d lookAheadTargetPos = {estimatedNextTurretPos.X(), m_allianceZoneTargetPose.Y(), m_allianceZoneTargetPose.Rotation()};
    double lookAheadTargetDistance = nextTurretToAllianceZoneDistance;
    double timeToReachLookAheadTarget = timeToReachAllianceZone;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeToReachLookAheadTarget = m_timeToReachTargetMap[lookAheadTargetDistance];
        lookAheadTargetPos = m_allianceZoneTargetPose + frc::Transform2d {0.0_m, 
                                                           units::meter_t(-ySpeed*timeToReachLookAheadTarget),
                                                           0.0_rad};
        lookAheadTargetDistance = (lookAheadTargetPos - estimatedNextTurretPos).Translation().Norm().value();
    }

    params.hoodAngle = m_hoodPosMap[lookAheadTargetDistance];
    if (params.hoodAngle < HoodConstants::Position::MIN)
    {
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_ALLIANCE_ZONE;
        params.hoodAngle = HoodConstants::Position::MIN;
    }
    else if (params.hoodAngle > HoodConstants::Position::MAX)
    {
        params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];

    params.lookAheadTargetTurretPos = WRAP_ANGLE_NEG_PI_TO_PI(m_allianceZoneTargetPose.Rotation().Radians().value() - estimatedNextTurretPos.Rotation().Radians().value());

    params.lookAheadTargetPos = lookAheadTargetPos;

    m_lastRobotPos = robotPos;
    m_lastTimestamp = timestamp;
}