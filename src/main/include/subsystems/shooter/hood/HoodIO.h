#pragma once

#include "units/voltage.h"
struct HoodIOInputs
{
    bool isMotorConnected = true;
    
    double motorAppliedVoltage = 0.0;
    double motorBusVoltage = 0.0;
    double motorCurrent = 0.0;
    double motorTemperature = 0.0;

    double hoodAngle = 0.0;
};


class HoodIO {
public:
    virtual ~HoodIO() = default;

    virtual void UpdateInputs(HoodIOInputs& inputs) = 0;

    virtual void SetVoltage(units::volt_t voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;

    virtual void ResetEncoder() = 0;
};