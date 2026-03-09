#include "subsystems/ShootParametersCalculator.h"

#include "frc/DriverStation.h"
#include "subsystems/turret/TurretConstants.h"
#include "subsystems/shooter/hood/HoodConstants.h"
#include "subsystems/shooter/flywheel/FlywheelConstants.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

ShootParametersCalculator::ShootParametersCalculator()
{
    m_hoodPosMap.insert(1.20, NDEGtoRAD(1.2));
    m_hoodPosMap.insert(2.535, NDEGtoRAD(12));
    m_hoodPosMap.insert(3.037, NDEGtoRAD(14));
    m_hoodPosMap.insert(5.163, NDEGtoRAD(19.1));

    m_flywheelSpeedMap.insert(1.20, 2200.0);
    m_flywheelSpeedMap.insert(2.535, 3150.0);
    m_flywheelSpeedMap.insert(3.037, 3350.0);
    m_flywheelSpeedMap.insert(5.163, 3580.0);

    m_timeToReachTargetMap.insert(1.20, 1.2); //tuneme
    m_timeToReachTargetMap.insert(2.535, 1.3); //tuneme
    m_timeToReachTargetMap.insert(3.037, 1.4); //tuneme
    m_timeToReachTargetMap.insert(5.163, 1.63);
}

void ShootParametersCalculator::SetAlliance(frc::DriverStation::Alliance alliance)
{
    if (alliance == frc::DriverStation::Alliance::kBlue)
    {
        m_pHubTargetPos = new frc::Pose2d(FieldConstants::Hub::BLUE_PLACEMENT);
        m_pAllianceZoneTargetPose = new frc::Pose2d(FieldConstants::AllianceZone::BLUE_CENTER_POSITION);
    }
    else
    {
        m_pHubTargetPos = new frc::Pose2d(FieldConstants::Hub::RED_PLACEMENT);
        m_pAllianceZoneTargetPose = new frc::Pose2d(FieldConstants::AllianceZone::RED_CENTER_POSITION);
    }
}

void ShootParametersCalculator::SetRobotPos(frc::Pose2d robotPos, double timestamp)
{
    m_pLastRobotPos = new frc::Pose2d(robotPos);
    m_lastTimestamp = timestamp;
}

void ShootParametersCalculator::CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{ 
    frc::Transform2d robotDisplacement = robotPos - *m_pLastRobotPos;
    double elapsedTime = m_lastTimestamp-timestamp;

    double xSpeed = robotDisplacement.X().value()/(elapsedTime);
    double ySpeed = robotDisplacement.Y().value()/(elapsedTime);
    double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(xSpeed * TIME_PER_CYCLE), 
                                                                  units::meter_t(ySpeed * TIME_PER_CYCLE),
                                                                  units::radian_t(rotationSpeed * TIME_PER_CYCLE)});

        
    frc::Pose2d turretPosInNextRobotFrame = {TurretConstants::Specifications::ROBOT_TO_TURRET.X(),
                                             TurretConstants::Specifications::ROBOT_TO_TURRET.Y(),
                                             units::radian_t{turretOrientation}};

    frc::Pose2d hubPosInRobotFrame = m_pHubTargetPos->RelativeTo(estimatedNextRobotPos);
    frc::Pose2d hubPosInNextTurretFrame = {(hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*cos(turretOrientation) + (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*sin(turretOrientation),
                                           -(hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*sin(turretOrientation) + (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*cos(turretOrientation),
                                           0.0_deg};

    double hubToTurretDistance = sqrt(pow(hubPosInNextTurretFrame.X().value(),2) + pow(hubPosInNextTurretFrame.Y().value(),2));
    double timeToReachHub = m_timeToReachTargetMap[hubToTurretDistance];

    frc::Pose2d *pCorrectedTargetPose = new frc::Pose2d(hubPosInNextTurretFrame);
    double correctedTargetToTurretDistance = hubToTurretDistance;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeToReachHub = m_timeToReachTargetMap[correctedTargetToTurretDistance];
        pCorrectedTargetPose = new frc::Pose2d(*pCorrectedTargetPose + frc::Transform2d {units::meter_t(-xSpeed*timeToReachHub), 
                                                                          units::meter_t(-ySpeed*timeToReachHub),
                                                                          0.0_rad});
        correctedTargetToTurretDistance = sqrt(pow(pCorrectedTargetPose->X().value(),2) + pow(pCorrectedTargetPose->Y().value(),2));
    }

    params.hoodAngle = m_hoodPosMap[correctedTargetToTurretDistance];
    if (params.hoodAngle < HoodConstants::Position::MIN)
    {
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_HUB;
        params.hoodAngle = HoodConstants::Position::MIN;
    }
    else if (params.hoodAngle > HoodConstants::Position::MAX)
    {
        params.flywheelSpeed = m_flywheelSpeedMap[correctedTargetToTurretDistance];
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[correctedTargetToTurretDistance];

    params.lookAheadTargetTurretPos = PIPI(pCorrectedTargetPose->RelativeTo(turretPosInNextRobotFrame).Translation().Angle().Radians().value());

    params.lookAheadTargetPos = *pCorrectedTargetPose;

    SetRobotPos(robotPos, timestamp);
}

void ShootParametersCalculator::CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{
    // frc::Transform2d robotDisplacement = robotPos - *m_pLastRobotPos;
    // double elapsedTime = m_lastTimestamp-timestamp;

    // double xSpeed = robotDisplacement.X().value()/(elapsedTime);
    // double ySpeed = robotDisplacement.Y().value()/(elapsedTime);
    // double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    // frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(xSpeed * TIME_PER_CYCLE), 
    //                                                               units::meter_t(ySpeed * TIME_PER_CYCLE),
    //                                                               units::radian_t(rotationSpeed * TIME_PER_CYCLE)});

    // frc::Pose2d turretPosInNextRobotFrame = {TurretConstants::Specifications::ROBOT_TO_TURRET.X(),
    //                                          TurretConstants::Specifications::ROBOT_TO_TURRET.Y(),
    //                                          units::radian_t{turretOrientation}};

    // frc::Pose2d allianceZonePosInRobotFrame = m_pHubTargetPos->RelativeTo(estimatedNextRobotPos);
    // frc::Pose2d allianceZonePosInNextTurretFrame = {(hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*cos(turretOrientation) - (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*sin(turretOrientation),
    //                                        (hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*sin(turretOrientation) + (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*cos(turretOrientation),
    //                                        0.0_deg};

    // double nextTurretToAllianceZoneDistance = (*m_pAllianceZoneTargetPose - estimatedNextTurretPos).Translation().Y().value();
    // double timeToReachAllianceZone = m_timeToReachTargetMap[nextTurretToAllianceZoneDistance];

    // frc::Pose2d lookAheadTargetPos = {estimatedNextTurretPos.X(), m_pAllianceZoneTargetPose->Y(), m_pAllianceZoneTargetPose->Rotation()};
    // double lookAheadTargetDistance = nextTurretToAllianceZoneDistance;
    // double timeToReachLookAheadTarget = timeToReachAllianceZone;

    // for (int i = 0; i < 20; i++) //TUNEME
    // {
    //     timeToReachLookAheadTarget = m_timeToReachTargetMap[lookAheadTargetDistance];
    //     lookAheadTargetPos = *m_pAllianceZoneTargetPose + frc::Transform2d {0.0_m, 
    //                                                        units::meter_t(-ySpeed*timeToReachLookAheadTarget),
    //                                                        0.0_rad};
    //     lookAheadTargetDistance = (lookAheadTargetPos - estimatedNextTurretPos).Translation().Norm().value();
    // }

    // params.hoodAngle = m_hoodPosMap[lookAheadTargetDistance];
    // if (params.hoodAngle < HoodConstants::Position::MIN)
    // {
    //     params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_ALLIANCE_ZONE;
    //     params.hoodAngle = HoodConstants::Position::MIN;
    // }
    // else if (params.hoodAngle > HoodConstants::Position::MAX)
    // {
    //     params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];
    //     params.hoodAngle = HoodConstants::Position::MAX;
    // }
    // else
    //     params.flywheelSpeed = m_flywheelSpeedMap[lookAheadTargetDistance];

    // params.lookAheadTargetTurretPos = WRAP_ANGLE_NEG_PI_TO_PI(m_pAllianceZoneTargetPose->Rotation().Radians().value() - estimatedNextTurretPos.Rotation().Radians().value());

    // params.lookAheadTargetPos = lookAheadTargetPos;

    // SetRobotPos(robotPos, timestamp);
}


double ShootParametersCalculator::PIPI(double angle_rad)
{
    double result = std::fmod(angle_rad,NF64_2PI);
    if (result < 0)
    {
        result += NF64_2PI;
    }
    if (result > NF64_PI)
    {
        result -= NF64_2PI;
    }
    return result;
}