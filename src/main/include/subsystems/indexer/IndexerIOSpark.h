#pragma once

#include "rev/SparkMax.h"


#include "indexerIO.h"
#include "indexerConstants.h"

class IndexerIOSpark  final : public IndexerIO
{
  private:
    rev::spark::SparkMax m_indexerMotor {IndexerConstants::indexerMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_indexerMotorConfig;
    rev::spark::SparkMax m_clodeMotor {IndexerConstants::clodeMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_clodeMotorConfig;

    rev::spark::SparkClosedLoopController m_indexerPIDFController{m_indexerMotor.GetClosedLoopController()};

  public:
    IndexerIOSpark();
    ~IndexerIOSpark() = default;

    void UpdateInputs(IndexerIOInputs& inputs) override;
    void SetVoltage(double voltage, double clodeVoltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle, double clodeVoltage) override; //COMMENTME
    void SetVelocity(double velocity, double clodeVoltage) override; 
};