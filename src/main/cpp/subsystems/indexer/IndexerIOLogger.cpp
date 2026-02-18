#include "subsystems/indexer/indexerIOLogger.h"

IndexerIOLogger::IndexerIOLogger(wpi::log::DataLog& log, const std::string& path)
    :               isindexerMotorConnected(log, path + "/indexerMotor" + "/isConnected"),
          indexerMotorCurrent(log, path + "/indexerMotor" + "/Current"),
          indexerMotorAppliedVoltage(log, path + "/indexerMotor" + "/AppliedVoltage"),
          indexerMotorBusVoltage(log, path + "/indexerMotor" + "/BusVoltage"),
          indexerMotorTemperature(log, path + "/indexerMotor" + "/Temperature"),
          isClodeMotorConnected(log, path + "/pivotMotor" + "/isConnected"),
          clodeMotorCurrent(log, path + "/pivotMotor" + "/Current"),
          clodeMotorAppliedVoltage(log, path + "/michelMotor" + "/AppliedVoltage"),
          clodeMotorBusVoltage(log, path + "/michelMotor" + "/BusVoltage"),
          clodeMotorTemperature(log, path + "/michelMotor" + "/Temperature"),
          
          m_path(path)
{}

void IndexerIOLogger::Log(const IndexerIOInputs& inputs) {
        isindexerMotorConnected.Append(inputs.isindexerMotorConnected);
    indexerMotorAppliedVoltage.Append(inputs.indexerMotorAppliedVoltage);
    indexerMotorBusVoltage.Append(inputs.indexerMotorBusVoltage);
    indexerMotorCurrent.Append(inputs.indexerMotorCurrent);
    indexerMotorTemperature.Append(inputs.indexerMotorTemperature);
    isClodeMotorConnected.Append(inputs.isClodeMotorConnected);
    clodeMotorAppliedVoltage.Append(inputs.clodeAppliedVoltage);
    clodeMotorBusVoltage.Append(inputs.clodeBusVoltage);
    clodeMotorCurrent.Append(inputs.clodeCurrent);
    clodeMotorTemperature.Append(inputs.clodeTemperature);
}