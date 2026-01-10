# QuestNav C++ Library

This directory contains the C++ implementation of the QuestNav library for FRC robots.

## Overview

The QuestNav C++ library provides a comprehensive interface for communicating with an Oculus/Meta Quest VR headset for robot localization and tracking in FRC robotics applications.

## Features

- Real-time pose tracking and localization data
- Command sending and response handling
- Device status monitoring (battery, tracking state, connectivity)
- NetworkTables-based communication protocol
- Full WPILib integration

## Installation

Add the QuestNav vendor dependency to your robot project:

1. Open your robot project in VS Code
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS)
3. Type "WPILib: Manage Vendor Libraries"
4. Select "Install new library (online)"
5. Enter the vendor URL: `https://maven.questnav.gg/releases/gg/questnav/questnavlib-json/2025-1.0.0/questnavlib-json-2025-1.0.0.json`

Or manually add to your `vendordeps/` directory.

## Usage

### Basic Example

```cpp
#include <questnav/QuestNav.h>
#include <frc/TimedRobot.h>
#include <frc/geometry/Pose2d.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // QuestNav instance is created automatically
  }

  void RobotPeriodic() override {
    // Process command responses
    m_questNav.CommandPeriodic();

    // Get latest pose data
    auto newFrames = m_questNav.GetAllUnreadPoseFrames();
    for (const auto& frame : newFrames) {
      // Use frame.questPose3d and frame.dataTimestamp with pose estimator
      if (m_questNav.IsTracking() && m_questNav.IsConnected()) {
        // Add to pose estimator
        // m_poseEstimator.AddVisionMeasurement(
        //     frame.questPose3d.ToPose2d(),
        //     units::second_t{frame.dataTimestamp},
        //     standardDeviations
        // );
      }
    }
  }

  void AutonomousInit() override {
    // Set initial robot pose (example: starting at origin)
    frc::Pose2d initialPose{1.0_m, 2.0_m, frc::Rotation2d{90_deg}};
    m_questNav.SetPose(initialPose);
  }

  void TeleopPeriodic() override {
    // Monitor connection and device status
    if (m_questNav.IsConnected()) {
      auto batteryPercent = m_questNav.GetBatteryPercent();
      if (batteryPercent && *batteryPercent < 20) {
        // Low battery warning
        frc::DriverStation::ReportWarning("QuestNav battery low!");
      }
    }
  }

 private:
  questnav::QuestNav m_questNav;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
```

### Advanced Usage with Pose Estimator

```cpp
#include <questnav/QuestNav.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>

class DriveSubsystem {
 public:
  DriveSubsystem() {
    // Initialize pose estimator with Quest measurements
  }

  void Periodic() {
    // Process Quest data
    m_questNav.CommandPeriodic();
    
    // Get all unread pose frames
    auto frames = m_questNav.GetAllUnreadPoseFrames();
    
    for (const auto& frame : frames) {
      // Only use data when Quest is tracking and connected
      if (m_questNav.IsTracking() && m_questNav.IsConnected()) {
        // Convert 3D pose to 2D for drive pose estimator
        auto pose2d = frame.questPose3d.ToPose2d();
        
        // Add vision measurement with appropriate standard deviations
        // Tune these values based on your robot and environment
        wpi::array<double, 3> stdDevs = {0.1, 0.1, 0.05};
        
        m_poseEstimator.AddVisionMeasurement(
            pose2d,
            units::second_t{frame.dataTimestamp},
            stdDevs
        );
      }
    }
    
    // Update pose estimator with other odometry data
    // ...
  }

  frc::Pose2d GetPose() const {
    return m_poseEstimator.GetEstimatedPosition();
  }

  void ResetPose(const frc::Pose2d& pose) {
    // Reset pose estimator
    m_poseEstimator.ResetPosition(/* ... */);
    
    // Also reset Quest tracking to match
    m_questNav.SetPose(pose);
  }

 private:
  questnav::QuestNav m_questNav;
  frc::SwerveDrivePoseEstimator m_poseEstimator{/* ... */};
};
```

## API Reference

### QuestNav Class

The main interface for communicating with the Quest headset.

#### Key Methods

- `void SetPose(const frc::Pose3d& pose)` - Sets the field-relative pose of the Quest
- `void SetPose(const frc::Pose2d& pose)` - Convenience method for 2D pose
- `std::vector<PoseFrame> GetAllUnreadPoseFrames()` - Gets all new pose data since last call
- `void CommandPeriodic()` - Processes command responses (call in RobotPeriodic)
- `bool IsConnected() const` - Checks if Quest is connected
- `bool IsTracking() const` - Checks if Quest is tracking
- `std::optional<int> GetBatteryPercent() const` - Gets battery level (0-100)
- `double GetLatency() const` - Gets communication latency in milliseconds

### PoseFrame Struct

Represents a single frame of pose tracking data.

#### Members

- `frc::Pose3d questPose3d` - The Quest's 3D pose
- `double dataTimestamp` - NetworkTables timestamp (use for pose estimation)
- `double appTimestamp` - Quest internal timestamp (debugging only)
- `int frameCount` - Sequential frame number
- `bool isTracking` - Whether Quest is tracking

## Coordinate System

QuestNav uses the WPILib field coordinate system:

- **X-axis**: Forward direction (towards opposing alliance)
- **Y-axis**: Left direction (when facing forward)
- **Z-axis**: Up direction
- **Rotation**: Counter-clockwise positive (standard mathematical convention)
- **Units**: Meters for translation, radians for rotation

## Important Notes

1. **Call CommandPeriodic()**: Must be called regularly (typically in RobotPeriodic) to process command responses
2. **Check Connection**: Always check `IsConnected()` and `IsTracking()` before using pose data
3. **Set Initial Pose**: Call `SetPose()` at the start of autonomous/teleop when the robot position is known
4. **Mounting Offset**: The pose from Quest is the Quest's pose, not the robot's. Apply mounting offset in your code
5. **Standard Deviations**: Tune measurement uncertainties based on your specific robot and environment

## Troubleshooting

### Quest not connecting
- Verify Quest is on same network as robot
- Check Ethernet adapter compatibility
- Ensure QuestNav app is running on Quest
- Check NetworkTables connection

### Poor tracking quality
- Improve lighting conditions
- Ensure environment has sufficient visual features
- Check for camera obstructions
- Reduce vibration on Quest mount

### Version mismatch warnings
- Update vendor dependency to match Quest app version
- Check that QuestNavLib version matches Quest app version

## Support

For questions, troubleshooting help, or to share your experiences with QuestNav, join the community discussion on our [Discord](https://discord.gg/Zfan2qgkRZ).

## License

This project is licensed under the MIT License.
