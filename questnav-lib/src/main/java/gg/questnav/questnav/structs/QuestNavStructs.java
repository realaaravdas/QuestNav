/*
 * QUESTNAV
 * https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 */

package gg.questnav.questnav.structs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * Container class for QuestNav data structures used with WPILib Structs.
 */
public class QuestNavStructs {
    private QuestNavStructs() {}

    /**
     * Data structure representing a single frame of tracking data from the Quest headset.
     */
    @StructSerializable
    public static class QuestNavFrameData implements StructSerializable {
        public int frameCount;
        public double timestamp;
        public Pose3d pose;
        public boolean isTracking;

        public QuestNavFrameData() {
            this.pose = new Pose3d();
        }

        public QuestNavFrameData(int frameCount, double timestamp, Pose3d pose, boolean isTracking) {
            this.frameCount = frameCount;
            this.timestamp = timestamp;
            this.pose = pose;
            this.isTracking = isTracking;
        }
    }

    /**
     * Data structure representing device status information.
     */
    @StructSerializable
    public static class QuestNavDeviceData implements StructSerializable {
        public int trackingLostCounter;
        public double batteryPercent;

        public QuestNavDeviceData() {}

        public QuestNavDeviceData(int trackingLostCounter, double batteryPercent) {
            this.trackingLostCounter = trackingLostCounter;
            this.batteryPercent = batteryPercent;
        }
    }

    /**
     * Data structure for sending commands to the Quest headset.
     */
    @StructSerializable
    public static class QuestNavCommand implements StructSerializable {
        public int type;
        public int commandId;
        public Pose3d targetPose;

        public QuestNavCommand() {
            this.targetPose = new Pose3d();
        }

        public QuestNavCommand(int type, int commandId, Pose3d targetPose) {
            this.type = type;
            this.commandId = commandId;
            this.targetPose = targetPose;
        }
    }

    /**
     * Data structure for responses to commands from the Quest headset.
     */
    @StructSerializable
    public static class QuestNavCommandResponse implements StructSerializable {
        public int commandId;
        public boolean success;
        public String errorMessage;

        public QuestNavCommandResponse() {
            this.errorMessage = "";
        }

        public QuestNavCommandResponse(int commandId, boolean success, String errorMessage) {
            this.commandId = commandId;
            this.success = success;
            this.errorMessage = errorMessage;
        }
    }

    public static class QuestNavCommandType {
        public static final int Unknown = 0;
        public static final int PoseReset = 1;
        public static final int ConfigUpdate = 2;
    }
}
