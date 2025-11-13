package com.team254.frc2025.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveIOInputsAutoLogged extends DriveIO.DriveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GyroAngle", gyroAngle);
    table.put("Pose", Pose);
    table.put("Speeds", Speeds);
    table.put("ModuleStates", ModuleStates);
    table.put("ModuleTargets", ModuleTargets);
    table.put("ModulePositions", ModulePositions);
    table.put("RawHeading", RawHeading);
    table.put("Timestamp", Timestamp);
    table.put("OdometryPeriod", OdometryPeriod);
    table.put("SuccessfulDaqs", SuccessfulDaqs);
    table.put("FailedDaqs", FailedDaqs);
  }

  @Override
  public void fromLog(LogTable table) {
    gyroAngle = table.get("GyroAngle", gyroAngle);
    Pose = table.get("Pose", Pose);
    Speeds = table.get("Speeds", Speeds);
    ModuleStates = table.get("ModuleStates", ModuleStates);
    ModuleTargets = table.get("ModuleTargets", ModuleTargets);
    ModulePositions = table.get("ModulePositions", ModulePositions);
    RawHeading = table.get("RawHeading", RawHeading);
    Timestamp = table.get("Timestamp", Timestamp);
    OdometryPeriod = table.get("OdometryPeriod", OdometryPeriod);
    SuccessfulDaqs = table.get("SuccessfulDaqs", SuccessfulDaqs);
    FailedDaqs = table.get("FailedDaqs", FailedDaqs);
  }

  public DriveIOInputsAutoLogged clone() {
    DriveIOInputsAutoLogged copy = new DriveIOInputsAutoLogged();
    copy.gyroAngle = this.gyroAngle;
    copy.Pose = this.Pose;
    copy.Speeds = this.Speeds;
    copy.ModuleStates = this.ModuleStates.clone();
    copy.ModuleTargets = this.ModuleTargets.clone();
    copy.ModulePositions = this.ModulePositions.clone();
    copy.RawHeading = this.RawHeading;
    copy.Timestamp = this.Timestamp;
    copy.OdometryPeriod = this.OdometryPeriod;
    copy.SuccessfulDaqs = this.SuccessfulDaqs;
    copy.FailedDaqs = this.FailedDaqs;
    return copy;
  }
}
