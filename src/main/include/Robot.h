// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <optional>
#include "RobotContainer.h"
#include "choreo/Choreo.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/ComplexStructLogger.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  
  void DriverStationConnected() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;  

  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;

  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
 private:
  // Trajectories should always be loaded at startup, not when the autonomous period begins. 
  //  Loading trajectories is a blocking operation, and larger trajectories may take multiple seconds to load on a RoboRIO, 
  //  cutting into the time a robot has to run its autonomous routine in a match.
 void LoadAutonomousTrajectories();
 
  RobotContainer m_container; 
  std::string gameData;

  StructArrayLogger<frc::Pose2d> trajectoryLogger{"/Trajectory/Robot"};
  choreo::Trajectory<choreo::DifferentialSample> trajectory;
  Alert m_isNotCompetitionRobot{"Not CompBot used", Alert::AlertType::WARNING};

  Alert m_pilot{"Pilot currently driving :", Alert::AlertType::INFO};
  Alert m_operator{"Operator currently operatoring :", Alert::AlertType::INFO};
  Alert m_robot{"Operator currently used :", Alert::AlertType::INFO};
};
