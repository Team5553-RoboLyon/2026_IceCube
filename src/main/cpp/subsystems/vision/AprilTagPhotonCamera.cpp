#include "subsystems/vision/AprilTagPhotonCamera.h"
#include "photon/PhotonUtils.h"

int AprilTagPhotonCamera::GetTargetID()
{
  if (m_pipelineResult.HasTargets())
  {
    return m_bestTarget->GetFiducialId();
  }
  return -1;
}

double AprilTagPhotonCamera::GetTargetPoseAmbiguity()
{
  return m_bestTarget->GetPoseAmbiguity();
}

double AprilTagPhotonCamera::GetTargetSkew()
{
  DEBUG_ASSERT(false, "GetTargetSkew() used with an AprilTagPhotonCamera");
  return 0;
}

double AprilTagPhotonCamera::AlternateHorizontalDistanceToTarget()
{
  if (m_pipelineResult.HasTargets())
  {
    frc::Transform3d robotToTarget = m_bestTarget->GetAlternateCameraToTarget()+m_cameraToTarget;
    return sqrt(pow(robotToTarget.X().value(),2)+pow(robotToTarget.X().value(),2));
  }
  return -1.0;
}

frc::Pose2d AprilTagPhotonCamera::GetAlternativeTargetPose() //TODO
{
  if (m_pRobotPos != nullptr && HasTarget())
  {
    frc::Transform3d transform3dToObject = m_bestTarget->GetAlternateCameraToTarget()+m_cameraToTarget;
    frc::Transform2d transform2dToObject{transform3dToObject.X(), transform3dToObject.Y(), transform3dToObject.Rotation().Z()};
    return *m_pRobotPos + transform2dToObject;
  }
  DEBUG_ASSERT(false,
              "GetTargetPose() maybe used with m_pRobotPos not initialized (or there is no target)");
  return ErrorPose2dOutput;
}