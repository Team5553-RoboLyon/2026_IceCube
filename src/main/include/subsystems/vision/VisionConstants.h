// #pragma once

// #include <frc/apriltag/AprilTagFieldLayout.h>
// #include <frc/apriltag/AprilTagFields.h>
// #include <frc/Filesystem.h>

// #include <frc/geometry/Rotation2d.h>
// #include <frc/geometry/Rotation3d.h>
// #include <frc/geometry/Transform3d.h>
// #include <frc/geometry/Translation3d.h>

// // #include <frc/units/angle.h>
// // #include "frc/units/length.h"

// // #include <photon/simulation/SimCameraProperties.h>

// #include <wpi/fs.h>

// #include <memory>
// #include <string>

// #include "subsystems/vision/VisionFilterParameters.h"

// namespace VisionConstants {

//     /* ---------------- AprilTag layouts ---------------- */

//     inline frc::AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT_2026 =
//         frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
        
//     constexpr const char* CUSTOM_APRIL_TAG_JSON = "2026-rebuilt.json";
        
//     inline const std::filesystem::path APRIL_TAG_DIR =
//     std::filesystem::path{frc::filesystem::GetDeployDirectory()} / "apriltag";

// inline const std::filesystem::path LAYOUT_FILE =
//     APRIL_TAG_DIR / CUSTOM_APRIL_TAG_JSON;

//     /**
//      * Loads a custom AprilTag layout from a JSON file. If loading fails, it
//      * defaults to the 2026 layout.
//      */
//     inline frc::AprilTagFieldLayout LoadCustomAprilTagLayout() {
//     try {
//         return frc::AprilTagFieldLayout{LAYOUT_FILE};
//     } catch (const std::exception& e) {
//         std::fprintf(stderr,
//                     "Error loading custom AprilTag field layout: %s\n"
//                     "Defaulting to 2026 field layout\n",
//                     e.what());
//         return APRIL_TAG_FIELD_LAYOUT_2026;
//     }
//     }

//     inline const frc::AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT_CUSTOM =
//         LoadCustomAprilTagLayout();

//     /* Choice of layout used */
//     inline const frc::AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
//         APRIL_TAG_FIELD_LAYOUT_2026;

//     /* ---------------- Vision tuning constants ---------------- */

//     inline constexpr double XY_STANDARD_DEV_BASE = 0.0060;
//     inline constexpr double ROTATION_STANDARD_DEV_BASE = 0.040;

//     inline constexpr units::millimeter_t APRILTAG_WIDTH = 165_mm;
//     inline constexpr double MAX_AMBIGUITY_RATIO = 0.5;
//     inline constexpr double MAX_APRILTAG_DISTANCE = 6.0;

//     inline constexpr units::centimeter_t Z_HEIGHT_MARGIN = 15_cm;

//     /* Estimation of the FOV for distance calculation */
//     inline const frc::Rotation2d ESTIMATED_FOV =
//         frc::Rotation2d(units::degree_t{60.0}); //TUNEME

//     /* ---------------- Robot → Camera transforms ---------------- */

//     inline const frc::Transform3d ROBOT_TO_CAMERA{ //TUNEME
//         frc::Translation3d{0.198_m, 0.221_m, 0.21386_m},
//         frc::Rotation3d{
//             0_deg,
//             units::degree_t{-20},
//             units::degree_t{-14}
//         }
//     };

//     /* ---------------- Camera names ---------------- */

//     constexpr const char* CAMERA_NAME  = "camera";

//     // /* ---------------- Camera simulation ---------------- */

//     // inline photon::SimCameraProperties CAMERA_SIM_PROPERTIES = [] {
//     // photon::SimCameraProperties props;
//     // props.SetCalibration(800, 600, frc::Rotation2d::FromDegrees(100));
//     // props.SetCalibError(0.25, 0.08);
//     // props.SetFPS(20);
//     // props.SetAvgLatency(35_ms);
//     // props.SetLatencyStdDev(5_ms);
//     // return props;
//     // }();

//     /* ---------------- Vision filter parameters ---------------- */

//     inline const VisionFilterParameters VISION_FILTER_PARAMETERS{
//         XY_STANDARD_DEV_BASE,
//         ROTATION_STANDARD_DEV_BASE,
//         APRILTAG_WIDTH,
//         MAX_AMBIGUITY_RATIO,
//         MAX_APRILTAG_DISTANCE,
//         ESTIMATED_FOV,
//         Z_HEIGHT_MARGIN,
//         APRIL_TAG_FIELD_LAYOUT
//     };

// } 