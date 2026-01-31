#pragma once

struct ClimberIOInputs
{
    bool isclimberMotorConnected = true;
    
    double climberMotorAppliedVoltage = 0.0;
    double climberMotorBusVoltage = 0.0;
    double climberMotorCurrent = 0.0;
    double climberMotorTemperature = 0.0;
    
};


class ClimberIO {
public:
    virtual ~ClimberIO() = default;

    virtual void UpdateInputs(ClimberIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;    
};