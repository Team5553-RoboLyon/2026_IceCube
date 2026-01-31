#include "subsystems/climber/climberIOLogger.h"

ClimberIOLogger::ClimberIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isclimberMotorConnected(log, path + "/climberMotor" + "/isConnected"),
          climberMotorCurrent(log, path + "/climberMotor" + "/Current"),
          climberMotorAppliedVoltage(log, path + "/climberMotor" + "/AppliedVoltage"),
          climberMotorBusVoltage(log, path + "/climberMotor" + "/BusVoltage"),
          climberMotorTemperature(log, path + "/climberMotor" + "/Temperature"),
          
          m_path(path)
{}

void ClimberIOLogger::Log(const ClimberIOInputs& inputs) {
        isclimberMotorConnected.Append(inputs.isclimberMotorConnected);
    climberMotorAppliedVoltage.Append(inputs.climberMotorAppliedVoltage);
    climberMotorBusVoltage.Append(inputs.climberMotorBusVoltage);
    climberMotorCurrent.Append(inputs.climberMotorCurrent);
    climberMotorTemperature.Append(inputs.climberMotorTemperature);
    
}