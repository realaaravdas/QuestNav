/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2026 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
package gg.questnav.questnav.structs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class QuestNavFrameData implements StructSerializable {
  public int frameCount;
  public double timestamp;
  public Pose3d pose;
  public boolean isTracking;

  public QuestNavFrameData(int frameCount, double timestamp, Pose3d pose, boolean isTracking) {
    this.frameCount = frameCount;
    this.timestamp = timestamp;
    this.pose = pose;
    this.isTracking = isTracking;
  }

  public QuestNavFrameData() {
    this.frameCount = 0;
    this.timestamp = 0;
    this.pose = new Pose3d();
    this.isTracking = false;
  }

  public static final Struct<QuestNavFrameData> struct =
      new Struct<>() {
        @Override
        public Class<QuestNavFrameData> getTypeClass() {
          return QuestNavFrameData.class;
        }

        @Override
        public String getTypeName() {
          return "QuestNavFrameData";
        }

        @Override
        public String getTypeString() {
          return "struct:QuestNavFrameData";
        }

        @Override
        public int getSize() {
          return Struct.kSizeInt32
              + Struct.kSizeDouble
              + Pose3d.struct.getSize()
              + Struct.kSizeBool;
        }

        @Override
        public String getSchema() {
          return "int32 frameCount;double timestamp;Pose3d pose;bool isTracking";
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {Pose3d.struct};
        }

        @Override
        public QuestNavFrameData unpack(ByteBuffer bb) {
          int frameCount = bb.getInt();
          double timestamp = bb.getDouble();
          Pose3d pose = Pose3d.struct.unpack(bb);
          boolean isTracking = bb.get() != 0;
          return new QuestNavFrameData(frameCount, timestamp, pose, isTracking);
        }

        @Override
        public void pack(ByteBuffer bb, QuestNavFrameData value) {
          bb.putInt(value.frameCount);
          bb.putDouble(value.timestamp);
          Pose3d.struct.pack(bb, value.pose);
          bb.put((byte) (value.isTracking ? 1 : 0));
        }
      };
}
