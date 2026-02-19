#include "subsystems/ShootParametersCalculator.h"

#include "frc/DriverStation.h"
#include "subsystems/turret/TurretConstants.h"
#include "subsystems/shooter/hood/HoodConstants.h"
#include "subsystems/shooter/flywheel/FlywheelConstants.h"
#include "Constants.h"

ShootParametersCalculator::ShootParametersCalculator()
{
    m_hoodPosMap.insert(0.0, 0.0);
    m_hoodPosMap.insert(6.0, HoodConstants::Position::MAX);

    m_flywheelSpeedMap.insert(0.0, 1706.0);
    m_flywheelSpeedMap.insert(6.0, 4500.0);

    m_timeToReachTargetMap.insert(0.0, 2.0);
    m_timeToReachTargetMap.insert(0.0, 6.0);
}

void ShootParametersCalculator::SetAlliance(frc::DriverStation::Alliance alliance)
{
    if (alliance == frc::DriverStation::Alliance::kBlue)
        m_targetPos = FieldConstants::Hub::BLUE_PLACEMENT;
    else
        m_targetPos = FieldConstants::Hub::RED_PLACEMENT;

}

void ShootParametersCalculator::SetRobotPos(frc::Pose2d robotPos, double timestamp)
{
    m_lastRobotPos = robotPos;
    m_lastTimestamp = timestamp;
}

void ShootParametersCalculator::CalculateNewParameters(ShootParameters& params, frc::Pose2d robotPos, double timestamp)
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

    double nextTurretToHubDistance = (m_targetPos - estimatedNextTurretPos).Translation().Norm().value();
    double timeToReachHub = m_timeToReachTargetMap[nextTurretToHubDistance];

    frc::Pose2d lookAheadTargetPos = m_targetPos;
    double lookAheadTargetDistance = nextTurretToHubDistance;
    double timeToReachLookAheadTarget = timeToReachHub;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeToReachLookAheadTarget = m_timeToReachTargetMap[lookAheadTargetDistance];
        lookAheadTargetPos = m_targetPos + frc::Transform2d {units::meter_t(-xSpeed*timeToReachLookAheadTarget), 
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
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_HUB;
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];

    params.lookAheadTargetTurretPos = m_targetPos.Rotation().Radians().value() - estimatedNextTurretPos.Rotation().Radians().value();

    m_lastRobotPos = robotPos;
    m_lastTimestamp = timestamp;       
}