#pragma once

struct ClimberIOInputs
{
    bool isMotorConnected = true;
    
    double motorAppliedVoltage = 0.0;
    double motorBusVoltage = 0.0;
    double motorCurrent = 0.0;
    double motorTemperature = 0.0;
    
    double climberHeight = 0.0;
    double hallEffectSensorValue = 0.0;
    bool bottomLimitSwitchValue = false;
};


class ClimberIO {
public:
    virtual ~ClimberIO() = default;

    virtual void UpdateInputs(ClimberIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;    

    virtual void ResetPosition() = 0;
};