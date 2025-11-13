package com.team254.frc2025.subsystems.indexer;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndexerSensorInputsAutoLogged extends IndexerSensorIO.IndexerSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("FirstIndexerBannerTriggered", firstIndexerBannerTriggered);
    table.put("SecondIndexerBannerTriggered", secondIndexerBannerTriggered);
  }

  @Override
  public void fromLog(LogTable table) {
    firstIndexerBannerTriggered = table.get("FirstIndexerBannerTriggered", firstIndexerBannerTriggered);
    secondIndexerBannerTriggered = table.get("SecondIndexerBannerTriggered", secondIndexerBannerTriggered);
  }

  public IndexerSensorInputsAutoLogged clone() {
    IndexerSensorInputsAutoLogged copy = new IndexerSensorInputsAutoLogged();
    copy.firstIndexerBannerTriggered = this.firstIndexerBannerTriggered;
    copy.secondIndexerBannerTriggered = this.secondIndexerBannerTriggered;
    return copy;
  }
}
