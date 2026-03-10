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

    m_flywheelSpeedMap.insert(1.20, 2500.0);
    m_flywheelSpeedMap.insert(2.535, 3200.0);
    m_flywheelSpeedMap.insert(3.037, 3450.0);
    m_flywheelSpeedMap.insert(5.163, 3500.0);

    m_timeToReachTargetMap.insert(1.20, 1.2); //tuneme
    m_timeToReachTargetMap.insert(2.535, 1.3); //tuneme
    m_timeToReachTargetMap.insert(3.037, 1.4); //tuneme
    m_timeToReachTargetMap.insert(5.163, 1.63);
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
    // m_logger.Log(m_hubTargetPos);
}

void ShootParametersCalculator::CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{ 
    m_logger.Log(m_hubTargetPos);

    frc::Transform2d robotDisplacement = robotPos - m_lastRobotPos;
    double elapsedTime = m_lastTimestamp-timestamp;

    double XSpeedInField = robotDisplacement.X().value()/(elapsedTime);
    double YSpeedInField = robotDisplacement.Y().value()/(elapsedTime);
    double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(XSpeedInField * TIME_PER_CYCLE), 
                                                                  units::meter_t(YSpeedInField * TIME_PER_CYCLE),
                                                                  units::radian_t(rotationSpeed * TIME_PER_CYCLE)});

        
    frc::Pose2d turretPosInNextRobotFrame = {TurretConstants::Specifications::ROBOT_TO_TURRET.X(),
                                             TurretConstants::Specifications::ROBOT_TO_TURRET.Y(),
                                             units::radian_t{turretOrientation}};

    frc::Pose2d hubPosInRobotFrame = m_hubTargetPos.RelativeTo(robotPos);
    frc::Pose2d hubPosInNextTurretFrame = {(hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*cos(turretOrientation) + (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*sin(turretOrientation),
                                           -(hubPosInRobotFrame.X()-turretPosInNextRobotFrame.X())*sin(turretOrientation) + (hubPosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*cos(turretOrientation),
                                           0.0_deg};

    // frc::Transform2d TurretInRobot = frc::Transform2d{frc::Pose2d{},turretPosInNextRobotFrame};
    // frc::Transform2d HubInTurret = frc::Transform2d{frc::Pose2d{},hubPosInNextTurretFrame};

    // frc::Pose2d hubInFieldForTurret = robotPos.TransformBy(TurretInRobot).TransformBy(HubInTurret);
    // m_projectedLogger.Log(hubInFieldForTurret);

    //Correction de la position du hub pour compenser de la vitesse du robot
    double hubToTurretDistance = sqrt(pow(hubPosInNextTurretFrame.X().value(),2) + pow(hubPosInNextTurretFrame.Y().value(),2));
    double timeToReachHub = m_timeToReachTargetMap[hubToTurretDistance];

    double XSpeedInRobotFrame = XSpeedInField*cos(robotPos.Rotation().Radians().value()); //There is no Y speed in robot frame because we're in a tank drivetrain

    double XSpeedInTurretFrame = XSpeedInRobotFrame*cos(turretOrientation);
    double YSpeedInTurretFrame = -XSpeedInRobotFrame*sin(turretOrientation);

    frc::Pose2d correctedTargetPose = hubPosInNextTurretFrame;
    double correctedTargetToTurretDistance = hubToTurretDistance;

    // for (int i = 0; i < 20; i++) //TUNEME
    // {
        timeToReachHub = m_timeToReachTargetMap[correctedTargetToTurretDistance];
        correctedTargetPose = {correctedTargetPose + frc::Transform2d {units::meter_t(-XSpeedInTurretFrame*timeToReachHub), 
                                                                          units::meter_t(-YSpeedInTurretFrame*timeToReachHub),
                                                                          0.0_rad}};
        correctedTargetToTurretDistance = sqrt(pow(correctedTargetPose.X().value(),2) + pow(correctedTargetPose.Y().value(),2));
    // }

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

    double turretTargetAngleOffset = atan2(correctedTargetPose.Y().value(), correctedTargetPose.X().value());

    params.lookAheadTargetTurretPos = PIPI(turretOrientation + turretTargetAngleOffset);

    params.lookAheadTargetPos = correctedTargetPose;

    SetRobotPos(robotPos, timestamp);

    //Log projected hub pos in field
    frc::Transform2d TurretInRobot = frc::Transform2d{frc::Pose2d{},turretPosInNextRobotFrame};
    frc::Transform2d HubInTurret = frc::Transform2d{frc::Pose2d{},correctedTargetPose};

    frc::Pose2d correctedHubInFieldForTurret = robotPos.TransformBy(TurretInRobot).TransformBy(HubInTurret);
    m_projectedLogger.Log(correctedHubInFieldForTurret);

    //log turret pos in field
    frc::Pose2d turretPosInField = robotPos.TransformBy(TurretInRobot);
    m_turretLogger.Log(turretPosInField);

    frc::SmartDashboard::PutBoolean("IsInRange", (params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT));
    params.isTargetInDeadZone = !(params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT);
}

void ShootParametersCalculator::CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{
 frc::Transform2d robotDisplacement = robotPos - m_lastRobotPos;
    double elapsedTime = m_lastTimestamp-timestamp;

    double XSpeedInField = robotDisplacement.X().value()/(elapsedTime);
    double YSpeedInField = robotDisplacement.Y().value()/(elapsedTime);
    double rotationSpeed = robotDisplacement.Rotation().Radians().value()/(elapsedTime);

    frc::Pose2d estimatedNextRobotPos = robotPos.Exp(frc::Twist2d{units::meter_t(XSpeedInField * TIME_PER_CYCLE), 
                                                                  units::meter_t(YSpeedInField * TIME_PER_CYCLE),
                                                                  units::radian_t(rotationSpeed * TIME_PER_CYCLE)});

        
    frc::Pose2d turretPosInNextRobotFrame = {TurretConstants::Specifications::ROBOT_TO_TURRET.X(),
                                             TurretConstants::Specifications::ROBOT_TO_TURRET.Y(),
                                             units::radian_t{turretOrientation}};

    frc::Pose2d allianceZonePosInRobotFrame = {};

    if(robotPos.Y() > FieldConstants::FIELD_WIDTH/2.0)
    {
        frc::Pose2d targetPos = {m_allianceZoneTargetPose.X(), 3.0*FieldConstants::FIELD_WIDTH/4.0, {}};
        m_logger.Log(targetPos);
        allianceZonePosInRobotFrame = targetPos.RelativeTo(estimatedNextRobotPos);
    }
    else
    {
        frc::Pose2d targetPos = {m_allianceZoneTargetPose.X(), FieldConstants::FIELD_WIDTH/4.0, {}};
        m_logger.Log(targetPos);
        allianceZonePosInRobotFrame = targetPos.RelativeTo(estimatedNextRobotPos);
    }
    frc::Pose2d hubPosInNextTurretFrame = {(allianceZonePosInRobotFrame.X()-turretPosInNextRobotFrame.X())*cos(turretOrientation) + (allianceZonePosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*sin(turretOrientation),
                                           -(allianceZonePosInRobotFrame.X()-turretPosInNextRobotFrame.X())*sin(turretOrientation) + (allianceZonePosInRobotFrame.Y()-turretPosInNextRobotFrame.Y())*cos(turretOrientation),
                                           0.0_deg};

    // frc::Transform2d TurretInRobot = frc::Transform2d{frc::Pose2d{},turretPosInNextRobotFrame};
    // frc::Transform2d AllianceZoneInTurret = frc::Transform2d{frc::Pose2d{},hubPosInNextTurretFrame};

    // frc::Pose2d AllianceZoneInFieldForTurret = robotPos.TransformBy(TurretInRobot).TransformBy(AllianceZoneInTurret);
    // m_projectedLogger.Log(AllianceZoneInFieldForTurret);

    //Correction de la position du hub pour compenser de la vitesse du robot
    double AllianceZoneToTurretDistance = sqrt(pow(hubPosInNextTurretFrame.X().value(),2) + pow(hubPosInNextTurretFrame.Y().value(),2));
    double timeToReachAllainceZone = m_timeToReachTargetMap[AllianceZoneToTurretDistance];

    double XSpeedInRobotFrame = XSpeedInField*cos(robotPos.Rotation().Radians().value()); //There is no Y speed in robot frame because we're in a tank drivetrain

    double XSpeedInTurretFrame = XSpeedInRobotFrame*cos(turretOrientation);
    double YSpeedInTurretFrame = -XSpeedInRobotFrame*sin(turretOrientation);

    frc::Pose2d correctedTargetPose = hubPosInNextTurretFrame;
    double correctedTargetToTurretDistance = AllianceZoneToTurretDistance;

    // for (int i = 0; i < 20; i++) //TUNEME
    // {
        timeToReachAllainceZone = m_timeToReachTargetMap[correctedTargetToTurretDistance];
        correctedTargetPose = {correctedTargetPose + frc::Transform2d {units::meter_t(-XSpeedInTurretFrame*timeToReachAllainceZone), 
                                                                          units::meter_t(-YSpeedInTurretFrame*timeToReachAllainceZone),
                                                                          0.0_rad}};
        correctedTargetToTurretDistance = sqrt(pow(correctedTargetPose.X().value(),2) + pow(correctedTargetPose.Y().value(),2));
    // }

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

    double turretTargetAngleOffset = atan2(correctedTargetPose.Y().value(), correctedTargetPose.X().value());

    params.lookAheadTargetTurretPos = PIPI(turretOrientation + turretTargetAngleOffset);

    params.lookAheadTargetPos = correctedTargetPose;

    SetRobotPos(robotPos, timestamp);

    //Log projected hub pos in field
    frc::Transform2d TurretInRobot = frc::Transform2d{frc::Pose2d{},turretPosInNextRobotFrame};
    frc::Transform2d AllianceZoneInTurret = frc::Transform2d{frc::Pose2d{},correctedTargetPose};

    frc::Pose2d correctedHubInFieldForTurret = robotPos.TransformBy(TurretInRobot).TransformBy(AllianceZoneInTurret);
    m_projectedLogger.Log(correctedHubInFieldForTurret);

    //log turret pos in field
    frc::Pose2d turretPosInField = robotPos.TransformBy(TurretInRobot);
    m_turretLogger.Log(turretPosInField);

    frc::SmartDashboard::PutBoolean("IsInRange", (params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT));
    params.isTargetInDeadZone = !(params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT && params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT);
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