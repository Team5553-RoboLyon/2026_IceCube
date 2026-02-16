#pragma once

#include "units/voltage.h"
struct TurretIOInputs
{
    bool isMotorConnected = true;
    
    double motorAppliedVoltage = 0.0;
    double motorBusVoltage = 0.0;
    double motorCurrent = 0.0;
    double motorTemperature = 0.0;
    double orientation = 0.0;

    double hallEffectSensorValue = 0.0;
};


class TurretIO {
public:
    virtual ~TurretIO() = default;

    virtual void UpdateInputs(TurretIOInputs& inputs) = 0;

    virtual void SetVoltage(units::volt_t voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;

    virtual void ResetOrientation() = 0;
};