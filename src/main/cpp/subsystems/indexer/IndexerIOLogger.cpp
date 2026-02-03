#include "subsystems/indexer/indexerIOLogger.h"

IndexerIOLogger::IndexerIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isindexerMotorConnected(log, path + "/indexerMotor" + "/isConnected"),
          indexerMotorCurrent(log, path + "/indexerMotor" + "/Current"),
          indexerMotorAppliedVoltage(log, path + "/indexerMotor" + "/AppliedVoltage"),
          indexerMotorBusVoltage(log, path + "/indexerMotor" + "/BusVoltage"),
          indexerMotorTemperature(log, path + "/indexerMotor" + "/Temperature"),
          
          m_path(path)
{}

void IndexerIOLogger::Log(const IndexerIOInputs& inputs) {
        isindexerMotorConnected.Append(inputs.isindexerMotorConnected);
    indexerMotorAppliedVoltage.Append(inputs.indexerMotorAppliedVoltage);
    indexerMotorBusVoltage.Append(inputs.indexerMotorBusVoltage);
    indexerMotorCurrent.Append(inputs.indexerMotorCurrent);
    indexerMotorTemperature.Append(inputs.indexerMotorTemperature);
    
}