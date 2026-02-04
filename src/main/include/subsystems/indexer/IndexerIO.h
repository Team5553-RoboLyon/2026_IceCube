#pragma once

struct IndexerIOInputs
{
        bool isindexerMotorConnected = true;
    
    double indexerMotorAppliedVoltage = 0.0;
    double indexerMotorBusVoltage = 0.0;
    double indexerMotorCurrent = 0.0;
    double indexerMotorTemperature = 0.0;
    
};


class IndexerIO {
public:
    virtual ~IndexerIO() = default;

    virtual void UpdateInputs(IndexerIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;

    
};