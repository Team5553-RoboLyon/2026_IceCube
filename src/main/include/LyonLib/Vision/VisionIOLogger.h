#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "VisionIO.h"

class VisionIOLogger {
public:
    VisionIOLogger(wpi::log::DataLog& log, const std::string& cameraName, const std::string& path);
    void Log(const VisionIO::VisionInputs& inputs);

private:
    wpi::log::BooleanLogEntry isConnected;
    wpi::log::BooleanLogEntry hasResults;
    wpi::log::DoubleLogEntry ambiguityRatio;
    wpi::log::IntegerArrayLogEntry visibleTagIDs;
    wpi::log::DoubleArrayLogEntry visibleTagArea;
    wpi::log::StructLogEntry<frc::Pose3d> BestFieldSpaceRobotPoses;
    wpi::log::StructLogEntry<frc::Pose3d> AlternatdFieldSpaceRobotPoses;
};