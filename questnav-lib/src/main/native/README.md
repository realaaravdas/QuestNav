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

### Manual Installation (Without Maven)

If you need to install the QuestNav C++ library manually without using the Maven repository (for example, if you're working offline or in an air-gapped environment), follow these steps:

#### Required Files

You'll need to copy the following files from this repository to your FRC robot project:

**Header Files** (copy to `src/main/native/include/`):
- `questnav-lib/src/main/native/include/questnav/QuestNav.h`
- `questnav-lib/src/main/native/include/questnav/PoseFrame.h`
- `questnav-lib/src/main/native/include/commands.pb.h`
- `questnav-lib/src/main/native/include/data.pb.h`
- `questnav-lib/src/main/native/include/geometry2d.pb.h`
- `questnav-lib/src/main/native/include/geometry3d.pb.h`

**Source Files** (copy to `src/main/native/cpp/`):
- `questnav-lib/src/main/native/cpp/QuestNav.cpp`
- `questnav-lib/src/main/native/cpp/commands.pb.cc`
- `questnav-lib/src/main/native/cpp/data.pb.cc`
- `questnav-lib/src/main/native/cpp/geometry2d.pb.cc`
- `questnav-lib/src/main/native/cpp/geometry3d.pb.cc`

#### Directory Structure

After copying, your FRC project should have this structure:

```
your-robot-project/
├── src/
│   └── main/
│       ├── cpp/
│       │   ├── Robot.cpp              # Your robot code
│       │   ├── QuestNav.cpp           # QuestNav implementation
│       │   ├── commands.pb.cc         # Protobuf generated code
│       │   ├── data.pb.cc
│       │   ├── geometry2d.pb.cc
│       │   └── geometry3d.pb.cc
│       └── native/
│           └── include/
│               ├── questnav/
│               │   ├── QuestNav.h     # Main QuestNav header
│               │   └── PoseFrame.h    # PoseFrame data structure
│               ├── commands.pb.h      # Protobuf generated headers
│               ├── data.pb.h
│               ├── geometry2d.pb.h
│               └── geometry3d.pb.h
└── build.gradle
```

#### Build Configuration

Add the following to your `build.gradle` file to configure the C++ build:

```gradle
nativeUtils.exportsConfigs {
    questnavlib {
        x86ExcludeSymbols = ['_CT??_R0?AV_System_error']
        x64ExcludeSymbols = ['_CT??_R0?AV_System_error']
    }
}

model {
    components {
        frcUserProgram(NativeExecutableSpec) {
            // ... your existing configuration ...
            
            binaries.all {
                // Add protobuf library linking
                if (it.targetPlatform.name == 'desktop') {
                    // For desktop simulation
                    lib library: 'protobuf', linkage: 'shared'
                } else {
                    // For RoboRIO
                    lib library: 'protobuf', linkage: 'static'
                }
            }
        }
    }
}
```

#### Required Dependencies

The QuestNav C++ library depends on:

1. **WPILib** (already included in FRC C++ projects)
   - `wpilibc`
   - `wpimath`
   - `ntcore`
   - `wpiutil`

2. **Protocol Buffers (protobuf)** - Required for NetworkTables communication
   - The protobuf library must be available on your system
   - For RoboRIO: Install via `roboRIO toolchain`
   - For desktop: Install via package manager (`apt install libprotobuf-dev` on Ubuntu, `brew install protobuf` on macOS)

#### Compilation Requirements

- **C++20 or later**: The library uses modern C++20 features
- **Compiler flags**: Add `-std=c++20` (GCC/Clang) or `/std:c++20` (MSVC) to your build configuration
- These flags should already be set in a standard FRC C++ project using GradleRIO 2025+

#### Installation Steps

1. **Download or clone this repository**:
   ```bash
   git clone https://github.com/realaaravdas/QuestNav.git
   cd QuestNav
   ```

2. **Copy header files to your project**:
   ```bash
   # From the QuestNav repository root
   cp -r questnav-lib/src/main/native/include/questnav/ your-robot-project/src/main/native/include/
   cp questnav-lib/src/main/native/include/*.pb.h your-robot-project/src/main/native/include/
   ```

3. **Copy source files to your project**:
   ```bash
   # From the QuestNav repository root
   cp questnav-lib/src/main/native/cpp/QuestNav.cpp your-robot-project/src/main/cpp/
   cp questnav-lib/src/main/native/cpp/*.pb.cc your-robot-project/src/main/cpp/
   ```

4. **Update your build.gradle** with the protobuf library configuration (see above)

5. **Build your robot project**:
   ```bash
   cd your-robot-project
   ./gradlew build
   ```

#### Verifying Installation

After installation, you should be able to include and use QuestNav in your robot code:

```cpp
#include <questnav/QuestNav.h>

// In your Robot class
questnav::QuestNav m_questNav;
```

If the build succeeds and you can access the QuestNav class, the manual installation was successful!

#### Troubleshooting Manual Installation

- **"protobuf library not found"**: Ensure protobuf is installed on your system and the library path is correctly configured in build.gradle
- **"QuestNav.h not found"**: Check that header files are in `src/main/native/include/questnav/`
- **"undefined reference to protobuf symbols"**: Verify protobuf library is linked in your build.gradle
- **C++20 compile errors**: Ensure your project is configured to use C++20 (standard in FRC 2025+)

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
