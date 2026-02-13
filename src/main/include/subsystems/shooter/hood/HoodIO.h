#pragma once

struct HoodIOInputs
{
    bool isHoodMotorConnected = true;
    
    double hoodMotorAppliedVoltage = 0.0;
    double hoodMotorBusVoltage = 0.0;
    double hoodMotorCurrent = 0.0;
    double hoodMotorTemperature = 0.0;

    double hoodPos = 0.0;
};


class HoodIO {
public:
    virtual ~HoodIO() = default;

    virtual void UpdateInputs(HoodIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;
};