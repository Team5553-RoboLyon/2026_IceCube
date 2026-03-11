#include "subsystems/ShootParametersCalculator.h"

#include "frc/DriverStation.h"
#include "subsystems/turret/TurretConstants.h"
#include "subsystems/shooter/hood/HoodConstants.h"
#include "subsystems/shooter/flywheel/FlywheelConstants.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

ShootParametersCalculator::ShootParametersCalculator()
{
    m_hoodPosMap.insert(0.87, NDEGtoRAD(1.3)); 
    m_hoodPosMap.insert(1.14, NDEGtoRAD(1.85)); 
    m_hoodPosMap.insert(1.72, NDEGtoRAD(2.95));
    // m_hoodPosMap.insert(1.12, NDEGtoRAD(1.2)); //tuneme
    m_hoodPosMap.insert(2.535, NDEGtoRAD(12)); //tuneme
    m_hoodPosMap.insert(3.037, NDEGtoRAD(14)); //tuneme
    m_hoodPosMap.insert(5.163, NDEGtoRAD(19.1)); //tuneme

    m_flywheelSpeedMap.insert(0.87, 2250.0);
    m_flywheelSpeedMap.insert(1.14, 2500.0);
    m_flywheelSpeedMap.insert(1.72, 2850.0);
    // m_flywheelSpeedMap.insert(1.25, 2500.0); //tuneme
    m_flywheelSpeedMap.insert(2.535, 3200.0); //tuneme
    m_flywheelSpeedMap.insert(3.037, 3450.0); //tuneme
    m_flywheelSpeedMap.insert(5.163, 3500.0); //tuneme

    m_timeToReachTargetMap.insert(1.25, 1.06);
    m_timeToReachTargetMap.insert(1.25, 1.04); 
    m_timeToReachTargetMap.insert(1.25, 1.25); 
    // m_timeToReachTargetMap.insert(1.25, 1.03); //tuneme
    m_timeToReachTargetMap.insert(2.535, 1.3); //tuneme
    m_timeToReachTargetMap.insert(3.037, 1.4); //tuneme
    m_timeToReachTargetMap.insert(5.163, 1.63); //tuneme
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

void ShootParametersCalculator::SetTurretPos(frc::Pose2d robotPos, double turretOrientation, double timestamp)
{
    frc::Transform2d turretPosInRobotFrame = {TurretConstants::Specifications::ROBOT_TO_TURRET.X(),
                                             TurretConstants::Specifications::ROBOT_TO_TURRET.Y(),
                                             units::radian_t{turretOrientation}};
    m_lastTurretPos = robotPos + turretPosInRobotFrame;
    m_lastTimestamp = timestamp;
    // m_logger.Log(m_hubTargetPos);
}

void ShootParametersCalculator::CalculateHubNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{ 
    m_logger.Log(m_hubTargetPos);

    vec2D robot;
    vec2D i_robot;
    vec2D j_robot;
    vec2D turret;
    vec2D i_turret;
    vec2D j_turret;
    vec2D turretTerrain;
    vec2D i_turretTerrain;
    vec2D j_turretTerrain;
    vec2D hub;

    //0 robot sur terrain
    robot.x = robotPos.X().value();
    robot.y = robotPos.Y().value();
    i_robot.x = cos(robotPos.Rotation().Radians().value());
    i_robot.y = sin(robotPos.Rotation().Radians().value());
    j_robot.x = -i_robot.y;
    j_robot.y = i_robot.x;

    //1 tourelle sur robot:
    turret.x = TurretConstants::Specifications::ROBOT_TO_TURRET.X().value();
    turret.y = TurretConstants::Specifications::ROBOT_TO_TURRET.Y().value();
    i_turret.x = cos(turretOrientation);
    i_turret.y = sin(turretOrientation);
    j_turret.x = -i_turret.y;
    j_turret.y = i_turret.x;

    //2 tourelle sur terrain:
    turretTerrain.x = robot.x + turret.x*i_robot.x + turret.y*j_robot.x;
    turretTerrain.y = robot.y + turret.x*i_robot.y + turret.y*j_robot.y;
    // i_turretTerrain.x = cos(turretOrientation);
    // i_turretTerrain.y = sin(turretOrientation);
    // j_turretTerrain.x = -i_robot.y;
    // j_turretTerrain.y = i_robot.x;

    //3 hub dans terrain
    hub.x = m_hubTargetPos.X().value();
    hub.y = m_hubTargetPos.Y().value();

    //4 vecteur tourelle-hub
    vec2D th;
    th.x = hub.x-turretTerrain.x;
    th.y = hub.y-turretTerrain.y;

    //5 vitesse tourelle
    double elapsedTime = timestamp-m_lastTimestamp;
    frc::SmartDashboard::PutNumber("elapsedTime", elapsedTime);

    vec2D vTurret;
    vTurret.x = (turretTerrain.x - m_lastTurretPos.X().value())/elapsedTime;
    vTurret.y = (turretTerrain.y - m_lastTurretPos.Y().value())/elapsedTime;

    //6 temps de vol
    double dth = sqrt(th.x*th.x + th.y*th.y);
    double timeOfFlight = m_timeToReachTargetMap[dth];
    frc::SmartDashboard::PutNumber("timeOfFlight", timeOfFlight);

    //7 projection
    vec2D projectedHub = hub;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeOfFlight = m_timeToReachTargetMap[dth];
        projectedHub.x = hub.x - timeOfFlight*vTurret.x;
        projectedHub.y = hub.y - timeOfFlight*vTurret.y;
        th.x = projectedHub.x-turretTerrain.x;
        th.y = projectedHub.y-turretTerrain.y;
        dth = sqrt(th.x*th.x + th.y*th.y);
    }

    //8 Angle de visee
    vec2D tp;
    tp.x = projectedHub.x-turretTerrain.x;
    tp.y = projectedHub.y-turretTerrain.y;
    double TurretAngleOnField = atan2(tp.y,tp.x);
    params.lookAheadTargetTurretPos = PIPI(TurretAngleOnField - robotPos.Rotation().Radians().value());

    //9 Autres parametres
    double dtp = sqrt(tp.x*tp.x + tp.y*tp.y);

    params.hoodAngle = m_hoodPosMap[dtp];
    if (params.hoodAngle < HoodConstants::Position::MIN)
    {
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_HUB;
        params.hoodAngle = HoodConstants::Position::MIN;
    }
    else if (params.hoodAngle > HoodConstants::Position::MAX)
    {
        params.flywheelSpeed = m_flywheelSpeedMap[dtp];
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[dtp];

    SetTurretPos(robotPos, turretOrientation, timestamp);

    //Log projected hub pos in field
    frc::Pose2d ProjectedHubPose {units::meter_t(projectedHub.x), units::meter_t(projectedHub.y),{}};
    m_projectedLogger.Log(ProjectedHubPose);

    //log turret pos in field;
    frc::Pose2d TurretInFieldPos {units::meter_t(turretTerrain.x), units::meter_t(turretTerrain.y),
                                    units::radian_t(robotPos.Rotation().Radians().value()+turretOrientation)};
    m_turretLogger.Log(TurretInFieldPos);

    frc::SmartDashboard::PutBoolean("IsInRange", (params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT));
    params.isTargetInDeadZone = !(params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT);

    params.turretInTolerance = (IS_IN_RANGE(turretOrientation, params.lookAheadTargetTurretPos, m_turretTolerance));
}

void ShootParametersCalculator::CalculateAllianceZoneNewParameters(ShootParameters& params, frc::Pose2d robotPos, double turretOrientation, double timestamp)
{
    m_logger.Log(m_hubTargetPos);

    vec2D robot;
    vec2D i_robot;
    vec2D j_robot;
    vec2D turret;
    vec2D i_turret;
    vec2D j_turret;
    vec2D turretTerrain;
    vec2D i_turretTerrain;
    vec2D j_turretTerrain;
    vec2D allianceZone;

    //0 robot sur terrain
    robot.x = robotPos.X().value();
    robot.y = robotPos.Y().value();
    i_robot.x = cos(robotPos.Rotation().Radians().value());
    i_robot.y = sin(robotPos.Rotation().Radians().value());
    j_robot.x = -i_robot.y;
    j_robot.y = i_robot.x;

    //1 tourelle sur robot:
    turret.x = TurretConstants::Specifications::ROBOT_TO_TURRET.X().value();
    turret.y = TurretConstants::Specifications::ROBOT_TO_TURRET.Y().value();
    i_turret.x = cos(turretOrientation);
    i_turret.y = sin(turretOrientation);
    j_turret.x = -i_turret.y;
    j_turret.y = i_turret.x;

    //2 tourelle sur terrain:
    turretTerrain.x = robot.x + turret.x*i_robot.x + turret.y*j_robot.x;
    turretTerrain.y = robot.y + turret.x*i_robot.y + turret.y*j_robot.y;
    // i_turretTerrain.x = cos(turretOrientation);
    // i_turretTerrain.y = sin(turretOrientation);
    // j_turretTerrain.x = -i_robot.y;
    // j_turretTerrain.y = i_robot.x;

    //3 alliance zone dans terrain
    allianceZone.x = m_allianceZoneTargetPose.X().value();
    if (turretTerrain.y < FieldConstants::FIELD_WIDTH.value()/2.0)
    {
        allianceZone.y = FieldConstants::FIELD_WIDTH.value()/4.0;
    }
    else
    {
        allianceZone.y = 3.0*FieldConstants::FIELD_WIDTH.value()/4.0;
    }


    //4 vecteur tourelle-alliance_zone
    vec2D ta;
    ta.x = allianceZone.x-turretTerrain.x;
    ta.y = allianceZone.y-turretTerrain.y;

    //5 vitesse tourelle
    double elapsedTime = timestamp-m_lastTimestamp;
    frc::SmartDashboard::PutNumber("elapsedTime", elapsedTime);

    vec2D vTurret;
    vTurret.x = (turretTerrain.x - m_lastTurretPos.X().value())/elapsedTime;
    vTurret.y = (turretTerrain.y - m_lastTurretPos.Y().value())/elapsedTime;

    //6 temps de vol
    double dta = sqrt(ta.x*ta.x + ta.y*ta.y);
    double timeOfFlight = m_timeToReachTargetMap[dta];
    frc::SmartDashboard::PutNumber("timeOfFlight", timeOfFlight);

    //7 projection
    vec2D projectedAllianceZone = allianceZone;

    for (int i = 0; i < 20; i++) //TUNEME
    {
        timeOfFlight = m_timeToReachTargetMap[dta];
        projectedAllianceZone.x = allianceZone.x - timeOfFlight*vTurret.x;
        projectedAllianceZone.y = allianceZone.y - timeOfFlight*vTurret.y;
        ta.x = projectedAllianceZone.x-turretTerrain.x;
        ta.y = projectedAllianceZone.y-turretTerrain.y;
        dta = sqrt(ta.x*ta.x + ta.y*ta.y);
    }

    //8 Angle de visee
    vec2D tp;
    tp.x = projectedAllianceZone.x-turretTerrain.x;
    tp.y = projectedAllianceZone.y-turretTerrain.y;
    double TurretAngleOnField = atan2(tp.y,tp.x);
    params.lookAheadTargetTurretPos = PIPI(TurretAngleOnField - robotPos.Rotation().Radians().value());

    //9 Autres parametres
    double dtp = sqrt(tp.x*tp.x + tp.y*tp.y);

    params.hoodAngle = m_hoodPosMap[dtp];
    if (params.hoodAngle < HoodConstants::Position::MIN)
    {
        params.flywheelSpeed = FlywheelConstants::Speed::AGAINST_HUB;
        params.hoodAngle = HoodConstants::Position::MIN;
    }
    else if (params.hoodAngle > HoodConstants::Position::MAX)
    {
        params.flywheelSpeed = m_flywheelSpeedMap[dtp];
        params.hoodAngle = HoodConstants::Position::MAX;
    }
    else
        params.flywheelSpeed = m_flywheelSpeedMap[dtp];

    SetTurretPos(robotPos, turretOrientation, timestamp);

    //Log projected hub pos in field
    frc::Pose2d ProjectedHubPose {units::meter_t(projectedAllianceZone.x), units::meter_t(projectedAllianceZone.y),{}};
    m_projectedLogger.Log(ProjectedHubPose);

    //log turret pos in field;
    frc::Pose2d TurretInFieldPos {units::meter_t(turretTerrain.x), units::meter_t(turretTerrain.y),
                                    units::radian_t(robotPos.Rotation().Radians().value()+turretOrientation)};
    m_turretLogger.Log(TurretInFieldPos);

    frc::SmartDashboard::PutBoolean("IsInRange", (params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT));
    params.isTargetInDeadZone = !(params.lookAheadTargetTurretPos > TurretConstants::Settings::BOTTOM_LIMIT &&  params.lookAheadTargetTurretPos < TurretConstants::Settings::TOP_LIMIT);

    params.turretInTolerance = (IS_IN_RANGE(turretOrientation, params.lookAheadTargetTurretPos, m_turretTolerance));
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