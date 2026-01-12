/*
* QUESTNAV
  https://github.com/QuestNav
* Copyright (C) 2026 QuestNav
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the MIT License as published.
*/
package gg.questnav.questnav.structs;

import edu.wpi.first.util.struct.StructSerializable;

public enum QuestNavCommandType implements StructSerializable {
  UNSPECIFIED(0),
  POSE_RESET(1);

  private final int value;

  QuestNavCommandType(int value) {
    this.value = value;
  }

  public int getValue() {
    return value;
  }

  public static QuestNavCommandType fromValue(int value) {
    for (QuestNavCommandType type : values()) {
      if (type.value == value) {
        return type;
      }
    }
    return UNSPECIFIED;
  }
}
