package com.team254.frc2025.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorSensorInputsAutoLogged extends ElevatorSensorIO.ElevatorSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorHallEffectTriggered", elevatorHallEffectTriggered);
    table.put("CanRangeDistanceLeft", canRangeDistanceLeft);
    table.put("CanRangeSignalStrengthLeft", canRangeSignalStrengthLeft);
    table.put("CanRangeDistanceRight", canRangeDistanceRight);
    table.put("CanRangeSignalStrengthRight", canRangeSignalStrengthRight);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorHallEffectTriggered = table.get("ElevatorHallEffectTriggered", elevatorHallEffectTriggered);
    canRangeDistanceLeft = table.get("CanRangeDistanceLeft", canRangeDistanceLeft);
    canRangeSignalStrengthLeft = table.get("CanRangeSignalStrengthLeft", canRangeSignalStrengthLeft);
    canRangeDistanceRight = table.get("CanRangeDistanceRight", canRangeDistanceRight);
    canRangeSignalStrengthRight = table.get("CanRangeSignalStrengthRight", canRangeSignalStrengthRight);
  }

  public ElevatorSensorInputsAutoLogged clone() {
    ElevatorSensorInputsAutoLogged copy = new ElevatorSensorInputsAutoLogged();
    copy.elevatorHallEffectTriggered = this.elevatorHallEffectTriggered;
    copy.canRangeDistanceLeft = this.canRangeDistanceLeft;
    copy.canRangeSignalStrengthLeft = this.canRangeSignalStrengthLeft;
    copy.canRangeDistanceRight = this.canRangeDistanceRight;
    copy.canRangeSignalStrengthRight = this.canRangeSignalStrengthRight;
    return copy;
  }
}
