/*
 * QUESTNAV
 * https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 */

#pragma once

#include <frc/geometry/Pose3d.h>
#include <wpi/struct/Struct.h>
#include <string>

namespace questnav {

/**
 * Data structure representing a single frame of tracking data from the Quest headset.
 */
struct QuestNavFrameData {
  int32_t frameCount = 0;
  double timestamp = 0.0;
  frc::Pose3d pose;
  bool isTracking = false;
};

/**
 * Data structure representing device status information.
 */
struct QuestNavDeviceData {
  int32_t trackingLostCounter = 0;
  double batteryPercent = 0.0;
};

/**
 * Enum representing command types sent to the Quest headset.
 */
enum class QuestNavCommandType : int32_t {
  Unknown = 0,
  PoseReset = 1,
  ConfigUpdate = 2
};

/**
 * Data structure for sending commands to the Quest headset.
 */
struct QuestNavCommand {
  QuestNavCommandType type = QuestNavCommandType::Unknown;
  int32_t commandId = 0;
  frc::Pose3d targetPose;
};

/**
 * Data structure for responses to commands from the Quest headset.
 */
struct QuestNavCommandResponse {
  int32_t commandId = 0;
  bool success = false;
  std::string errorMessage; // Fixed length not strictly required but safer for serialization if bounded, but Struct supports string
};

}  // namespace questnav

// WPILib Struct Traits Definitions
// These allow the structs to be serialized/deserialized automatically by NetworkTables

template <>
struct wpi::Struct<questnav::QuestNavFrameData> {
  static constexpr std::string_view GetTypeString() { return "struct:QuestNavFrameData"; }
  static constexpr size_t GetSize() { 
    return sizeof(int32_t) + sizeof(double) + wpi::Struct<frc::Pose3d>::GetSize() + sizeof(bool);
  }
  static constexpr std::string_view GetSchema() {
    return "int32 frameCount; double timestamp; Pose3d pose; bool isTracking";
  }

  static questnav::QuestNavFrameData Unpack(std::span<const uint8_t> data) {
    questnav::QuestNavFrameData out;
    out.frameCount = wpi::UnpackStruct<int32_t, 0>(data);
    out.timestamp = wpi::UnpackStruct<double, 4>(data);
    out.pose = wpi::UnpackStruct<frc::Pose3d, 12>(data);
    out.isTracking = wpi::UnpackStruct<bool, 12 + wpi::Struct<frc::Pose3d>::GetSize()>(data);
    return out;
  }

  static void Pack(std::span<uint8_t> data, const questnav::QuestNavFrameData& value) {
    wpi::PackStruct<int32_t, 0>(data, value.frameCount);
    wpi::PackStruct<double, 4>(data, value.timestamp);
    wpi::PackStruct<frc::Pose3d, 12>(data, value.pose);
    wpi::PackStruct<bool, 12 + wpi::Struct<frc::Pose3d>::GetSize()>(data, value.isTracking);
  }
};

template <>
struct wpi::Struct<questnav::QuestNavDeviceData> {
  static constexpr std::string_view GetTypeString() { return "struct:QuestNavDeviceData"; }
  static constexpr size_t GetSize() { return sizeof(int32_t) + sizeof(double); }
  static constexpr std::string_view GetSchema() {
    return "int32 trackingLostCounter; double batteryPercent";
  }

  static questnav::QuestNavDeviceData Unpack(std::span<const uint8_t> data) {
    questnav::QuestNavDeviceData out;
    out.trackingLostCounter = wpi::UnpackStruct<int32_t, 0>(data);
    out.batteryPercent = wpi::UnpackStruct<double, 4>(data);
    return out;
  }

  static void Pack(std::span<uint8_t> data, const questnav::QuestNavDeviceData& value) {
    wpi::PackStruct<int32_t, 0>(data, value.trackingLostCounter);
    wpi::PackStruct<double, 4>(data, value.batteryPercent);
  }
};

template <>
struct wpi::Struct<questnav::QuestNavCommandType> {
  static constexpr std::string_view GetTypeString() { return "int32"; }
  static constexpr size_t GetSize() { return sizeof(int32_t); }
  static constexpr std::string_view GetSchema() { return "int32"; }

  static questnav::QuestNavCommandType Unpack(std::span<const uint8_t> data) {
    return static_cast<questnav::QuestNavCommandType>(wpi::UnpackStruct<int32_t, 0>(data));
  }

  static void Pack(std::span<uint8_t> data, const questnav::QuestNavCommandType& value) {
    wpi::PackStruct<int32_t, 0>(data, static_cast<int32_t>(value));
  }
};

template <>
struct wpi::Struct<questnav::QuestNavCommand> {
  static constexpr std::string_view GetTypeString() { return "struct:QuestNavCommand"; }
  static constexpr size_t GetSize() { 
    return sizeof(int32_t) + sizeof(int32_t) + wpi::Struct<frc::Pose3d>::GetSize();
  }
  static constexpr std::string_view GetSchema() {
    return "int32 type; int32 commandId; Pose3d targetPose";
  }

  static questnav::QuestNavCommand Unpack(std::span<const uint8_t> data) {
    questnav::QuestNavCommand out;
    out.type = static_cast<questnav::QuestNavCommandType>(wpi::UnpackStruct<int32_t, 0>(data));
    out.commandId = wpi::UnpackStruct<int32_t, 4>(data);
    out.targetPose = wpi::UnpackStruct<frc::Pose3d, 8>(data);
    return out;
  }

  static void Pack(std::span<uint8_t> data, const questnav::QuestNavCommand& value) {
    wpi::PackStruct<int32_t, 0>(data, static_cast<int32_t>(value.type));
    wpi::PackStruct<int32_t, 4>(data, value.commandId);
    wpi::PackStruct<frc::Pose3d, 8>(data, value.targetPose);
  }
};

template <>
struct wpi::Struct<questnav::QuestNavCommandResponse> {
  static constexpr std::string_view GetTypeString() { return "struct:QuestNavCommandResponse"; }
  static constexpr size_t GetSize() { return kVariableSize; } // String makes it variable size
  static constexpr std::string_view GetSchema() {
    return "int32 commandId; bool success; string errorMessage";
  }

  static questnav::QuestNavCommandResponse Unpack(std::span<const uint8_t> data) {
    questnav::QuestNavCommandResponse out;
    size_t offset = 0;
    out.commandId = wpi::UnpackStruct<int32_t, 0>(data);
    offset += 4;
    out.success = wpi::UnpackStruct<bool, 4>(data);
    offset += 1;
    out.errorMessage = wpi::UnpackStruct<std::string, 5>(data.subspan(offset));
    return out;
  }

  static void Pack(std::vector<uint8_t>& data, const questnav::QuestNavCommandResponse& value) {
    wpi::PackStruct<int32_t>(data, value.commandId);
    wpi::PackStruct<bool>(data, value.success);
    wpi::PackStruct<std::string>(data, value.errorMessage);
  }
};
