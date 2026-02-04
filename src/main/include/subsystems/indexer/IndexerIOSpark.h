#pragma once

#include "rev/SparkMax.h"


#include "indexerIO.h"
#include "indexerConstants.h"

class IndexerIOSpark  final : public IndexerIO
{
  private:
        rev::spark::SparkMax m_indexerMotor { IndexerConstants::indexerMotor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_indexerMotorConfig;


    

  public:
    IndexerIOSpark();
    ~IndexerIOSpark() = default;

    void UpdateInputs(IndexerIOInputs& inputs) override;
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME

    
};