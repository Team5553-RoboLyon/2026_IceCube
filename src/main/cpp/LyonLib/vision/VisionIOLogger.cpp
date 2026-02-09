#include "LyonLib/Vision/VisionIOLogger.h"

VisionIOLogger::VisionIOLogger(wpi::log::DataLog& log, const std::string& cameraName, const std::string& path)
    :     isConnected(log, path + "/" + cameraName + "/isConnected"),
          hasResults(log, path + "/" + cameraName + "/hasResults"),
          ambiguityRatio(log, path + "/" + cameraName + "/ambiguityRatio"),
          visibleTagIDs(log, path + "/" + cameraName + "/visibleTagIDs"),
          visibleTagArea(log, path + "/" + cameraName + "/visibleTagArea"),
          BestFieldSpaceRobotPoses(log, path + "/" + cameraName + "/BestFieldSpaceRobotPoses"),
          AlternatdFieldSpaceRobotPoses(log, path + "/" + cameraName + "/AlternatdFieldSpaceRobotPoses")
{}

void VisionIOLogger::Log(const VisionIO::VisionInputs& inputs) {
    isConnected.Append(inputs.connected);
    hasResults.Append(inputs.hasResults);
    ambiguityRatio.Append(inputs.ambiguityRatio);
    visibleTagIDs.Append(inputs.visibleTagIDs);
    visibleTagArea.Append(inputs.tagAreas);
    BestFieldSpaceRobotPoses.Append(inputs.fieldSpaceRobotPoses[0]);
    AlternatdFieldSpaceRobotPoses.Append(inputs.fieldSpaceRobotPoses[1]);
}