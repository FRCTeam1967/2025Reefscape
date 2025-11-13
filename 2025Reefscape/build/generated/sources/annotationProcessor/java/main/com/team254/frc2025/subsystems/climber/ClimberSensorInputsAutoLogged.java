package com.team254.frc2025.subsystems.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberSensorInputsAutoLogged extends ClimberSensorIO.ClimberSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimberLeftLimitSwitchTriggered", climberLeftLimitSwitchTriggered);
    table.put("ClimberRightLimitSwitchTriggered", climberRightLimitSwitchTriggered);
  }

  @Override
  public void fromLog(LogTable table) {
    climberLeftLimitSwitchTriggered = table.get("ClimberLeftLimitSwitchTriggered", climberLeftLimitSwitchTriggered);
    climberRightLimitSwitchTriggered = table.get("ClimberRightLimitSwitchTriggered", climberRightLimitSwitchTriggered);
  }

  public ClimberSensorInputsAutoLogged clone() {
    ClimberSensorInputsAutoLogged copy = new ClimberSensorInputsAutoLogged();
    copy.climberLeftLimitSwitchTriggered = this.climberLeftLimitSwitchTriggered;
    copy.climberRightLimitSwitchTriggered = this.climberRightLimitSwitchTriggered;
    return copy;
  }
}
