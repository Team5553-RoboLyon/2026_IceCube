#pragma once

struct IntakeIOInputs
{
    bool isIntakeMotorConnected = true;
    
    double intakeMotorAppliedVoltage = 0.0;
    double intakeMotorBusVoltage = 0.0;
    double intakeMotorCurrent = 0.0;
    double intakeMotorTemperature = 0.0;
    bool isPivotMotorConnected = true;
    
    double pivotMotorAppliedVoltage = 0.0;
    double pivotMotorBusVoltage = 0.0;
    double pivotMotorCurrent = 0.0;
    double pivotMotorTemperature = 0.0;
    
    double pivotPos = 0.0;
};


class IntakeIO {
public:
    virtual ~IntakeIO() = default;

    virtual void UpdateInputs(IntakeIOInputs& inputs) = 0;

    virtual void SetVoltage(double intakeVoltage, double pivotVoltage) = 0; 
    virtual void SetDutyCycle(double intakeDutyCycle, double pivotDutyCycle) = 0;

    virtual void ResetEncoder() = 0;
};