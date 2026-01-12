/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2026 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
package gg.questnav.questnav.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class QuestNavDeviceData implements StructSerializable {
  public int trackingLostCounter;
  public int batteryPercent;

  public QuestNavDeviceData(int trackingLostCounter, int batteryPercent) {
    this.trackingLostCounter = trackingLostCounter;
    this.batteryPercent = batteryPercent;
  }

  public static final Struct<QuestNavDeviceData> struct =
      new Struct<>() {
        @Override
        public Class<QuestNavDeviceData> getTypeClass() {
          return QuestNavDeviceData.class;
        }

        @Override
        public String getTypeName() {
          return "QuestNavDeviceData";
        }

        @Override
        public String getTypeString() {
          return "struct:QuestNavDeviceData";
        }

        @Override
        public int getSize() {
          return Struct.kSizeInt32 + Struct.kSizeInt32;
        }

        @Override
        public String getSchema() {
          return "int32 trackingLostCounter;int32 batteryPercent";
        }

        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {};
        }

        @Override
        public QuestNavDeviceData unpack(ByteBuffer bb) {
          int trackingLostCounter = bb.getInt();
          int batteryPercent = bb.getInt();
          return new QuestNavDeviceData(trackingLostCounter, batteryPercent);
        }

        @Override
        public void pack(ByteBuffer bb, QuestNavDeviceData value) {
          bb.putInt(value.trackingLostCounter);
          bb.putInt(value.batteryPercent);
        }
      };
}
