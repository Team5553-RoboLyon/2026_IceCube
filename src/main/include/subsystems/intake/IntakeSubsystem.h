// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "roller/RollerConstants.h"
#include "pivot/PivotConstants.h"
#include "intakeIOLogger.h"
#include "roller/RollerIO.h"
#include "pivot/PivotIO.h"
#include "Constants.h"

#include "LyonLib/control/RateLimiter.h"
#include "LyonLib/control/pidRBL.h"
#include "LyonLib/logging/Alert.h"
#include "LyonLib/logging/TunableValueLogger.h"

class IntakeSubsystem : public frc2::SubsystemBase {
  public:
    IntakeSubsystem(RollerIO *pRollerIO, PivotIO *pPivotIO);

    enum class WantedState 
    {
      STAND_BY = 0, // no wanted state scheduled. (It's all good man, it's all good !)
      REFUEL = 1,
      EJECT = 2,
      BECOME_AN_INDEXER = 3,
      EXTEND = 4,
      RETURN_AT_HOME = 5,
      PROTECT_YOURSELF_AGAINST_EVIL_PILOT = 6
    };

    enum class SystemState
    {
      IDLE = 0,
      //Steady states
      STAYING_AT_HOME = 1,
      CHILLING_OUT = 2,
      REFUELING = 3,
      EJECTING = 4,
      FEELING_LIKE_AN_INDEXER = 5,
      PROTECTED_AGAINST_EVIL_PILOT = 6,
      //Transition states
      EXTENDING = 7,
      COMING_BACK_HOME = 8,
      GOING_TO_SAFE_POS = 9,
    };

    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();
    void SetControlMode(const ControlMode pivotMode, const ControlMode rollerControlMode);
    void SetPivotControlMode(const ControlMode mode);
    void SetRollerControlMode(const ControlMode mode);
    ControlMode GetPivotControlMode();
    ControlMode GetRollerControlMode();
    void TogglePivotControlMode();
    void ToggleRollerControlMode();

    void ActualisePIDCoef();

    bool IsOut();
    bool IsPivotMoving();
    bool IsInitialized() { return m_isInitialized; }
    // void ResetPivotEncoder();
    void SetManualControlInput(const double PivotInput, const double RollerInput);
    // void SetManualControlInput(const std::function<double()> Axis); //temporary


    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      RollerIO *m_pRollerIO;
      RollerIOInputs rollerInputs;
      PivotIO *m_pPivotIO;
      PivotIOInputs pivotInputs;
      IntakeIOLogger m_logger{frc::DataLogManager::GetLog(), "/Intake"};
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_pivotControlMode = PivotConstants::MainControlMode;
      ControlMode m_rollerControlMode = RollerConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_pivotPIDController;
    // === Control Inputs / Outputs ===
      double m_rollerOutput{0.0};
      double m_pivotOutput{0.0};
      double m_pivotManualControlInput{0.0};
      double m_rollerManualControlInput{0.0};
      double m_pivotTargetPos{0.0};
      double m_timestamp{0.0};
    // std::function<double()> m_fxAxis; //temporary
      TunableValueLogger m_tunableRollerVoltageLogger{"Intake/RollerVoltage", 8.0}; //RPM
      TunableValueLogger m_tunableRollerP{"Intake/P", 0.0};
      TunableValueLogger m_tunableRollerI{"Intake/i", 0.0};
      TunableValueLogger m_tunableRollerD{"Intake/D", 0.0};
    // === Status Flags ===
      bool m_isInitialized = true;
    // === System Alerts ===
      Alert m_rollerMotorDisconnected{"Roller motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_rollerMotorHot{"Roller motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_rollerMotorOverheating{"Roller motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_pivotMotorDisconnected{"Pivot motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_pivotMotorHot{"Pivot motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_pivotMotorOverheating{"Pivot motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_pivotLeftEncoderDisconnected{"Pivot Left encoder : Disconnected", Alert::AlertType::ERROR};
      Alert m_pivotRightEncoderDisconnected{"Pivot Right encoder : Disconnected", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};