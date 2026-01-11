// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

Robot::Robot() {}
void Robot::RobotInit() {
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  m_pilot.Set(true);
  m_pilot.SetText(std::string("Pilot currently driving : ") + PilotToString());
  m_operator.Set(true);
  m_operator.SetText(std::string("Operator currently operatoring : ") + OperatorToString());
  m_robot.Set(true);
  m_robot.SetText(std::string("Robot used : ") + RobotToString());
  m_isNotCompetitionRobot.Set(ROBOT_MODEL != COMPETITON);
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DriverStationConnected() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TeleopExit() {}

void Robot::DisabledInit() 
{
  m_container.shooterSubsystem.SetControlMode(ControlMode::DISABLED);
}
void Robot::DisabledPeriodic() {
}
void Robot::DisabledExit() {
  m_container.shooterSubsystem.SetControlMode(ShooterConstants::MainControlMode);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::TestExit() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
