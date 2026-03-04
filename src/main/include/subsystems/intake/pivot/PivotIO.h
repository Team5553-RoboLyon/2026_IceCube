#pragma once

struct PivotIOInputs
{
    bool isPivotMotorConnected = true;
    
    double pivotMotorAppliedVoltage = 0.0;
    double pivotMotorBusVoltage = 0.0;
    double pivotMotorCurrent = 0.0;
    double pivotMotorTemperature = 0.0;

    bool isLeftEncoderConnected = true;
    // bool isRightEncoderConnected = true;
    
    double pivotPos = 0.0;
};

class PivotIO {
public:
    virtual ~PivotIO() = default;

    virtual void UpdateInputs(PivotIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;
};