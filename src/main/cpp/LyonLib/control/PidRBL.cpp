#include "LyonLib/control/PidRBL.h"
#include "LyonLib/utils/MacroUtilsRBL.h"
#include "LyonLib/logging/DebugUtils.h"

PidRBL::PidRBL() 
{
    SetGains(0.0, 0.0, 0.0);
}
PidRBL::PidRBL(const double kp, const double ki, const double kd) 
{
    SetGains(kp, ki, kd);
}

void PidRBL::SetGains(const double kp, const double ki, const double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    if (m_ki != 0.0) {
        m_kaw = 1.0 / m_ki; // Set anti-windup gain based on integral gain
    }
    else {
        m_kaw = 0.0; // No anti-windup if integral gain is zero
    }
    Reset();
}

void PidRBL::SetSetpoint(const double setpoint)
{
    if(m_isInputLimitsActive) 
    {
        if(setpoint > m_inputMax)
            m_setpoint = m_inputMax;
        else if(setpoint < m_inputMin)
            m_setpoint = m_inputMin;
        else 
            m_setpoint = setpoint;
    }
    else 
    {
        m_setpoint = setpoint;
    }
}

void PidRBL::SetOutputLimits(const double min, const double max)
{
    m_outputMin = NMIN(min, max);
    m_outputMax = NMAX(min, max);
    DEBUG_ASSERT(m_outputMin <= m_outputMax, "PidRBL : Output minimum must be less than or equal to output maximum.");
}

void PidRBL::SetInputLimits(const double min, const double max)
{
    m_inputMin = NMIN(min, max);
    m_inputMax = NMAX(min, max);
    DEBUG_ASSERT(m_inputMin <= m_inputMax, "PidRBL : Input minimum must be less than or equal to input maximum.");
}

void PidRBL::SetInputLimits(const bool isActive)
{
    m_isInputLimitsActive = isActive;
}

void PidRBL::SetContinuous(const bool isContinuous)
{
    m_isContinuous = isContinuous;
}

double PidRBL::GetKP() const
{
    return m_kp;
}

double PidRBL::GetKI() const
{
    return m_ki;
}

double PidRBL::GetKD() const
{
    return m_kd;
}

double PidRBL::GetError() const
{
    return m_currentError;
}

double PidRBL::GetSetpoint() const
{
    return m_setpoint;
}

std::string PidRBL::GetState() const
{
    std::string state = "PID State: ";
    state += "Kp: " + std::to_string(m_kp) + "\n";
    state += "Ki: " + std::to_string(m_ki) + "\n";
    state += "Kd: " + std::to_string(m_kd) + "\n";
    state += "Kaw: " + std::to_string(m_kaw) + "\n";
    state += "Setpoint: " + std::to_string(m_setpoint) + "\n";
    state += "Input Min: " + std::to_string(m_inputMin) + "\n";
    state += "Input Max: " + std::to_string(m_inputMax) + "\n";
    state += "Current Error: " + std::to_string(m_currentError) + "\n";
    state += "Output: " + std::to_string(m_output) + "\n";
    state += "Output Min: " + std::to_string(m_outputMin) + "\n";
    state += "Output Max: " + std::to_string(m_outputMax) + "\n";
    state += "Last Timestamp: " + std::to_string(m_lastTimestamp) + "\n";
    state += "Delta Time: " + std::to_string(m_dt) + "\n";
    state += "Is Continuous: " + std::string(m_isContinuous ? "true" : "false") + "\n";
    state += "Integrative: " + std::to_string(m_integrative) + "\n";
    state += "Previous Measurement: " + std::to_string(m_previousMeasurement) + "\n";
    return state;
}

double PidRBL::CalculateWithRealTime(const double measurement, const double timestamp) {
    // Recalculate dt based on timestamps for real-time systems
    m_dt = timestamp - m_lastTimestamp;
    
    if(m_dt <= 0.0) 
    {
        // If time has not advanced, return the previous output without recalculating
        DEBUG_ASSERT(false, "PID : Timestamp must be greater than the last timestamp.");
        return 0.0;
    }

    m_lastTimestamp = timestamp;

    m_currentError = m_setpoint - measurement;
    
    if(m_isContinuous)
    {
        if(NABS(m_currentError) > (m_inputMax - m_inputMin) / 2.0)
        {
            if(m_currentError > 0.0)
            {
                m_currentError -= (m_inputMax - m_inputMin);
            }
            else
            {
                m_currentError += (m_inputMax - m_inputMin);
            }
        }
    }
    m_integrative += m_currentError * m_dt;          

    double unsaturated = m_kp * m_currentError + 
                            m_ki * m_integrative + 
                            (-m_kd * ((measurement - m_previousMeasurement) / m_dt));

    

    m_previousMeasurement = measurement; // Update previous measurement for next derivative calculation            

    // Clamp output within allowed range
    if (unsaturated > m_outputMax)
        m_output = m_outputMax;
    else if (unsaturated < m_outputMin)
        m_output = m_outputMin;
    else
        m_output = unsaturated;

    // === ANTI-WINDUP BACK-CALCULATION : accumulate integrative error only if the output is not saturated
    m_integrative += m_kaw * (m_output - unsaturated) * m_dt;
    return m_output;
}

double PidRBL::CalculateWithRealTime(const double setpoint, const double measurement, const double timestamp) {
    SetSetpoint(setpoint);
    return CalculateWithRealTime(measurement, timestamp);
}

double PidRBL::Calculate(const double measurement)
{
    return CalculateWithRealTime(measurement, m_lastTimestamp + THEORETICAL_DT);
}

double PidRBL::Calculate(const double setpoint, const double measurement) {
    SetSetpoint(setpoint);
    return CalculateWithRealTime(measurement, m_lastTimestamp + THEORETICAL_DT);
}
void PidRBL::Reset()
{
    m_setpoint = m_setpoint - m_currentError; // Reset setpoint to current measurement 
    m_previousMeasurement = m_setpoint; // Set previous measurement to new setpoint to avoid derivative kick
    m_currentError = 0.0;
    m_output = 0.0;
    ResetIntegrative();
}

void PidRBL::Reset(const double timestamp)
{
    m_lastTimestamp = timestamp;
    Reset();
}

void PidRBL::ResetIntegrative()
{
    m_integrative = 0.0;
}