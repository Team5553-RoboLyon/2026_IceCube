#pragma once

struct FlywheelIOInputs
{
    bool isLeftMotorConnected = true;
    
    double leftMotorAppliedVoltage = 0.0;
    double leftMotorBusVoltage = 0.0;
    double leftMotorCurrent = 0.0;
    double leftMotorTemperature = 0.0;
    double leftMotorEncoderVelocity = 0.0;

    bool isRightMotorConnected = true;
    
    double rightMotorAppliedVoltage = 0.0;
    double rightMotorBusVoltage = 0.0;
    double rightMotorCurrent = 0.0;
    double rightMotorTemperature = 0.0;
    double rightMotorEncoderVelocity = 0.0;
    
    double ShooterVelocity = 0.0;
};


class FlywheelIO {
public:
    virtual ~FlywheelIO() = default;

    virtual void UpdateInputs(FlywheelIOInputs& inputs) = 0;

    virtual void SetVoltage(double voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;
};