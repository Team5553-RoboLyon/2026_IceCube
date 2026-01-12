#pragma once

struct IntakeIOInputs
{
    bool isleftMotorConnected = true;
    
    double leftMotorAppliedVoltage = 0.0;
    double leftMotorBusVoltage = 0.0;
    double leftMotorCurrent = 0.0;
    double leftMotorTemperature = 0.0;
    bool isrightMotorConnected = true;
    
    double rightMotorAppliedVoltage = 0.0;
    double rightMotorBusVoltage = 0.0;
    double rightMotorCurrent = 0.0;
    double rightMotorTemperature = 0.0;
    
};


class IntakeIO {
public:
    virtual ~IntakeIO() = default;

    virtual void UpdateInputs(IntakeIOInputs& inputs) = 0;

    virtual void SetVoltage(double leftVoltage, double rightVoltage) = 0; 
    virtual void SetDutyCycle(double leftDutyCycle, double rightDutyCycle) = 0;

    
};