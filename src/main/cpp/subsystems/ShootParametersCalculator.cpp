#include "subsystems/ShootParametersCalculator.h"

#include "frc/DriverStation.h"
#include "subsystems/turret/TurretConstants.h"
#include "Constants.h"

ShootParametersCalculator::ShootParametersCalculator(frc::Pose2d robotStartingPos)
{
    robotPos = robotStartingPos;
}

void ShootParametersCalculator::SetAlliance(frc::DriverStation::Alliance alliance)
{
    InBlueAlliance = (alliance == frc::DriverStation::Alliance::kBlue);
}

void ShootParametersCalculator::CalculateNewParameters(ShootParameters& params, frc::Pose2d newRobotPos)
{ 
    frc::Transform2d robotDisplacement = newRobotPos - robotPos;

    if (InBlueAlliance)
    {
        params.lookAheadTargetPos = FieldConstants::Hub::BLUE_PLACEMENT + robotDisplacement.Inverse();
        params.lookAheadTargetTurretPos = WRAP_ANGLE_0_TO_2PI((FieldConstants::Hub::BLUE_PLACEMENT.Rotation().Radians() - robotPos.Rotation().Radians()).value());
    }
    else
    {
        params.lookAheadTargetPos = FieldConstants::Hub::RED_PLACEMENT + robotDisplacement.Inverse();
        params.lookAheadTargetTurretPos = WRAP_ANGLE_0_TO_2PI((FieldConstants::Hub::RED_PLACEMENT.Rotation().Radians() - robotPos.Rotation().Radians()).value());
    }

    frc::Transform2d robotToLookAheadPos = (newRobotPos + TurretConstants::Specifications::ROBOT_TO_TURRET)-params.lookAheadTargetPos;
    params.lookAheadTargetDistance = sqrt(pow(robotToLookAheadPos.X().value(),2)+pow(robotToLookAheadPos.Y().value(),2));
}