#include "LyonLib/control/FeedForwardModel.h"
#include "LyonLib/utils/MacroUtilsRBL.h"
#include "LyonLib/logging/DebugUtils.h"

FeedForwardModel::FeedForwardModel() 
{
    SetGains(0.0, 0.0, 0.0);
}
FeedForwardModel::FeedForwardModel(const double kS, const double kV, const double kA) 
{
    SetGains(kS, kV, kA);
}

void FeedForwardModel::SetGains(const double kS, const double kV, const double kA, const std::function<double(double)>& kG)
{
    m_kS = kS;
    m_kV = kV;
    m_kA = kA;
    m_kG = kG;
}

void FeedForwardModel::SetOutputLimits(const double min, const double max)
{
    m_outputMin = NMIN(min, max);
    m_outputMax = NMAX(min, max);
    DEBUG_ASSERT(m_outputMin <= m_outputMax, "FeedForwardModel : Output minimum must be less than or equal to output maximum.");
}

double FeedForwardModel::GetKS() const
{
    return m_kS;
}

double FeedForwardModel::GetKV() const
{
    return m_kV;
}

double FeedForwardModel::GetKA() const
{
    return m_kA;
}

double FeedForwardModel::GetKG(const double position) const
{
    return m_kG(position);
}

std::string FeedForwardModel::GetState() const
{
    std::string state = " FeedForwardModel State: ";
    state += "KS: " + std::to_string(m_kS) + "\n";
    state += "KV: " + std::to_string(m_kV) + "\n";
    state += "KA: " + std::to_string(m_kA) + "\n";

    if (m_kG) {
        state += "KG(0.0): " + std::to_string(m_kG(0.0)) + "\n";
    } else {
        state += "KG: <none>\n";
    }
    
    state += "Output: " + std::to_string(m_output) + "\n";
    state += "Output Min: " + std::to_string(m_outputMin) + "\n";
    state += "Output Max: " + std::to_string(m_outputMax) + "\n";
    return state;
}

double FeedForwardModel::Calculate(const double position, const double velocity, const double acceleration) {      
    m_output = m_kS * NSIGN(velocity) + 
                    m_kV * velocity + 
                    m_kA * acceleration + 
                    m_kG(position);

    // Clamp output within allowed range
    if (m_output > m_outputMax)
        m_output = m_outputMax;
    else if (m_output < m_outputMin)
        m_output = m_outputMin;
    return m_output;
}