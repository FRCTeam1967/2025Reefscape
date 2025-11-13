package com.team254.frc2025.subsystems.claw;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClawSensorInputsAutoLogged extends ClawSensorIO.ClawSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("StageCoralBannerTriggered", stageCoralBannerTriggered);
    table.put("ScoreCoralBannerTriggered", scoreCoralBannerTriggered);
  }

  @Override
  public void fromLog(LogTable table) {
    stageCoralBannerTriggered = table.get("StageCoralBannerTriggered", stageCoralBannerTriggered);
    scoreCoralBannerTriggered = table.get("ScoreCoralBannerTriggered", scoreCoralBannerTriggered);
  }

  public ClawSensorInputsAutoLogged clone() {
    ClawSensorInputsAutoLogged copy = new ClawSensorInputsAutoLogged();
    copy.stageCoralBannerTriggered = this.stageCoralBannerTriggered;
    copy.scoreCoralBannerTriggered = this.scoreCoralBannerTriggered;
    return copy;
  }
}
