#pragma once

struct RollerIOInputs
{
    bool isRollerMotorConnected = true;
    
    double rollerMotorAppliedVoltage = 0.0;
    double rollerMotorBusVoltage = 0.0;
    double rollerMotorCurrent = 0.0;
    double rollerMotorTemperature = 0.0;
    
};


class RollerIO {
public:
    virtual ~RollerIO() = default;

    virtual void UpdateInputs(RollerIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;
};