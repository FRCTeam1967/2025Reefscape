package com.team254.lib.subsystems;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CanCoderInputsAutoLogged extends CanCoderInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AbsolutePositionRotations", absolutePositionRotations);
    table.put("VelocityRotations", velocityRotations);
  }

  @Override
  public void fromLog(LogTable table) {
    absolutePositionRotations = table.get("AbsolutePositionRotations", absolutePositionRotations);
    velocityRotations = table.get("VelocityRotations", velocityRotations);
  }

  public CanCoderInputsAutoLogged clone() {
    CanCoderInputsAutoLogged copy = new CanCoderInputsAutoLogged();
    copy.absolutePositionRotations = this.absolutePositionRotations;
    copy.velocityRotations = this.velocityRotations;
    return copy;
  }
}
