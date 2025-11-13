package com.team254.lib.subsystems;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorInputsAutoLogged extends MotorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VelocityUnitsPerSecond", velocityUnitsPerSecond);
    table.put("UnitPosition", unitPosition);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentStatorAmps", currentStatorAmps);
    table.put("CurrentSupplyAmps", currentSupplyAmps);
    table.put("RawRotorPosition", rawRotorPosition);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityUnitsPerSecond = table.get("VelocityUnitsPerSecond", velocityUnitsPerSecond);
    unitPosition = table.get("UnitPosition", unitPosition);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentStatorAmps = table.get("CurrentStatorAmps", currentStatorAmps);
    currentSupplyAmps = table.get("CurrentSupplyAmps", currentSupplyAmps);
    rawRotorPosition = table.get("RawRotorPosition", rawRotorPosition);
  }

  public MotorInputsAutoLogged clone() {
    MotorInputsAutoLogged copy = new MotorInputsAutoLogged();
    copy.velocityUnitsPerSecond = this.velocityUnitsPerSecond;
    copy.unitPosition = this.unitPosition;
    copy.appliedVolts = this.appliedVolts;
    copy.currentStatorAmps = this.currentStatorAmps;
    copy.currentSupplyAmps = this.currentSupplyAmps;
    copy.rawRotorPosition = this.rawRotorPosition;
    return copy;
  }
}
