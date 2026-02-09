#include "subsystems/climber/climberIOLogger.h"

ClimberIOLogger::ClimberIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isMotorConnected(log, path + "/climberMotor" + "/isConnected"),
          motorCurrent(log, path + "/climberMotor" + "/Current"),
          motorAppliedVoltage(log, path + "/climberMotor" + "/AppliedVoltage"),
          motorBusVoltage(log, path + "/climberMotor" + "/BusVoltage"),
          motorTemperature(log, path + "/climberMotor" + "/Temperature"),
          
          climberHeight(log, path + "/climber" + "/Height"),
          hallEffectSensorValue(log, path + "/climber" + "/HallEffectSensorValue"),
          bottomLimitSwitchValue(log, path + "/climber" + "/BottomLimitSwitchValue"),
          
          m_path(path)
{}

void ClimberIOLogger::Log(const ClimberIOInputs& inputs) {
    isMotorConnected.Append(inputs.isMotorConnected);
    motorAppliedVoltage.Append(inputs.motorAppliedVoltage);
    motorBusVoltage.Append(inputs.motorBusVoltage);
    motorCurrent.Append(inputs.motorCurrent);
    motorTemperature.Append(inputs.motorTemperature);
    
    climberHeight.Append(inputs.climberHeight);
    hallEffectSensorValue.Append(inputs.hallEffectSensorValue);
    bottomLimitSwitchValue.Append(inputs.bottomLimitSwitchValue);
}