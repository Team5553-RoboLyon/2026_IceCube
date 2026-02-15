#include "subsystems/climber/climberIOLogger.h"

ClimberIOLogger::ClimberIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isMotorConnected(log, path + "/Motor" + "/IO/isConnected"),
          motorCurrent(log, path + "/Motor" + "/Current"),
          motorAppliedVoltage(log, path + "/Motor" + "/AppliedVoltage"),
          motorBusVoltage(log, path + "/Motor" + "/BusVoltage"),
          motorTemperature(log, path + "/Motor" + "/Temperature"),
          
          hammerHeight(log, path + "/Sensors" + "/HammerHeight"),
          irbreakerValue(log, path + "/Sensors" + "/IRBreakerValue"),
          bottomLimitSwitchValue(log, path + "/Sensors" + "/BottomLimitSwitchValue"),
          
          m_path(path)
{}

void ClimberIOLogger::Log(const ClimberIOInputs& inputs) {
    isMotorConnected.Append(inputs.isMotorConnected);
    motorAppliedVoltage.Append(inputs.motorAppliedVoltage);
    motorBusVoltage.Append(inputs.motorBusVoltage);
    motorCurrent.Append(inputs.motorCurrent);
    motorTemperature.Append(inputs.motorTemperature);
    
    hammerHeight.Append(inputs.hammerHeight);
    irbreakerValue.Append(inputs.irbreakerValue);
    bottomLimitSwitchValue.Append(inputs.bottomLimitSwitchValue);
}