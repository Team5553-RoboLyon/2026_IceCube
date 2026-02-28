#pragma once

struct IndexerIOInputs
{
        bool isindexerMotorConnected = true;
    
    double indexerMotorAppliedVoltage = 0.0;
    double indexerMotorBusVoltage = 0.0;
    double indexerMotorCurrent = 0.0;
    double indexerMotorTemperature = 0.0;

    bool isClodeMotorConnected = true;

    double clodeAppliedVoltage = 0.0;
    double clodeBusVoltage = 0.0;
    double clodeCurrent = 0.0;
    double clodeTemperature = 0.0;
};


class IndexerIO {
public:
    virtual ~IndexerIO() = default;

    virtual void UpdateInputs(IndexerIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage, double clodeVoltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle, double clodeDutyCycle) = 0;

    
};