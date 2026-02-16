// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "LyonLib/logging/DebugUtils.h" 
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>

Robot::Robot() {}
void Robot::RobotInit() {
  //start data logging
  frc::DataLogManager::Start();
  //
  wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());

  m_pilot.Set(true);
  m_pilot.SetText(std::string("Pilot currently driving : ") + PilotToString());
  m_operator.Set(true);
  m_operator.SetText(std::string("Operator currently operatoring : ") + OperatorToString());
  m_robot.Set(true);
  m_robot.SetText(std::string("Robot used : ") + RobotToString());
  m_isNotCompetitionRobot.Set(ROBOT_MODEL != COMPETITON);

  LoadAutonomousTrajectories();
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  gameData = frc::DriverStation::GetGameSpecificMessage();
  if(gameData.length() > 0)
  {
    switch (gameData[0])
    {
      case 'B' :
        break;
      case 'R' :
        break;
      default :
        DEBUG_ASSERT(false, "Unexpected first character of game data received");
        break;
    }
  } else {
    //Code for no data received yet
  }
}

void Robot::DriverStationConnected() {
  //link driver station to data logging
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void Robot::AutonomousInit() {
  //TODO : set pose
  m_container.drivetrain.ResetOdometryPose(trajectory.GetInitialSample().value().GetPose());
  m_container.drivetrain.SetWantedDrive(DriveMode::AUTO_PATH_FOLLOWER);
  m_container.drivetrain.SetDesiredAutoTrajectory(trajectory);

}
void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  //TODO : set pose
  m_container.drivetrain.SetWantedDrive(DriveMode::ARCADE_DRIVE);
}
void Robot::TeleopPeriodic() {}
void Robot::TeleopExit() {}

void Robot::DisabledInit() {
  m_container.drivetrain.SetWantedDrive(DriveMode::DISABLE);
}
void Robot::DisabledPeriodic() {}
void Robot::DisabledExit() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {
  m_container.drivetrain.SetWantedDrive(driveConstants::desiredDriveControl);
}
void Robot::TestExit() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::LoadAutonomousTrajectories() {
 trajectory = choreo::Choreo::LoadTrajectory<choreo::DifferentialSample>("Virgule").value(); 
 trajectoryLogger.Log(trajectory.GetPoses());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
