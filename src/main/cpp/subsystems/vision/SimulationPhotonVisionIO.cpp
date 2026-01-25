// #include "subsystems/vision/SimulationPhotonVisionIO.h"
// #include "subsystems/vision/VisionEnvironmentSimulator.h"

// SimulatedPhotonVisionIO::SimulatedPhotonVisionIO(
//     const std::string& cameraName,
//     const frc::Transform3d& robotToCameraTransform,
//     const frc::AprilTagFieldLayout& aprilTagFieldLayout,
//     const photon::SimCameraProperties& simCameraProperties)
//     : PhotonVisionIO(cameraName, robotToCameraTransform, aprilTagFieldLayout) {

//   // Create the PhotonVision simulation object for this camera
//   photon::PhotonCameraSim camSim{camera, simCameraProperties};

//   // Enable the processed and raw streams to localhost:1182.
//   camSim.EnableRawStream(true);
//   camSim.EnableProcessedStream(true);

//   // Enable drawing a wireframe visualization of the field to the camera streams.
//   // This is extremely resource-intensive and is disabled by default.
//   camSim.EnableDrawWireframe(true);

//   // Register this simulated camera with the field simulator.
//   VisionEnvironmentSimulator::GetInstance().AddCamera(camSim, robotToCameraTransform);
// }

//create a VisionEnvironmentSimulator.cpp file and implement the singleton pattern there