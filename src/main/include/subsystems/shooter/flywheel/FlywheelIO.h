#pragma once

#include "units/angular_velocity.h"
#include "units/voltage.h"
struct FlywheelIOInputs
{
    bool isLeftMotorConnected = true;
    
    double leftMotorAppliedVoltage = 0.0;
    double leftMotorBusVoltage = 0.0;
    double leftMotorCurrent = 0.0;
    double leftMotorTemperature = 0.0;
    double leftMotorInternalEncoderVelocity = 0.0;

    bool isRightMotorConnected = true;
    
    double rightMotorAppliedVoltage = 0.0;
    double rightMotorBusVoltage = 0.0;
    double rightMotorCurrent = 0.0;
    double rightMotorTemperature = 0.0;
    double rightMotorInternalEncoderVelocity = 0.0;
    
    double shooterVelocity = 0.0;
};


class FlywheelIO {
public:
    virtual ~FlywheelIO() = default;

    virtual void UpdateInputs(FlywheelIOInputs& inputs) = 0;

    virtual void SetVoltage(units::volt_t voltage) = 0; 
    virtual void SetDutyCycle(double dutyCycle) = 0;
    virtual void SetVelocity(units::angular_velocity::revolutions_per_minute_t velocity) = 0;
};