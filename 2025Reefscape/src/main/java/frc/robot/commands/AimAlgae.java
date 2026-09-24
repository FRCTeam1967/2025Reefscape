// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;

public class AimAlgae extends Command {
  public final CommandSwerveDrivetrain swerve;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Swapped FieldCentricFacingAngle -> FieldCentric.
  // FieldCentricFacingAngle expects an absolute field-relative heading to hold,
  // which is what getAngleToHub() gave you. tx from the neural detector is a
  // relative offset, not a heading, so it needs to drive a rotational RATE
  // instead — same pattern as algae_aim_proportional().
  private final SwerveRequest.FieldCentric driveAtAngle = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1) // 0.1 = deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AimAlgae(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double algae_aim_proportional() {
    double kP = 0.035;
    double targetingAngularVelocity = 0.0;

    if (LimelightHelpers.getTV("limelight-front")) {
      String detectedClass = LimelightHelpers.getDetectorClass("limelight-front");

      if (detectedClass != null && detectedClass.equals("algae")) {
        targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;
      }
    }

    targetingAngularVelocity *= MaxAngularRate;
    return targetingAngularVelocity;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setControl(
      driveAtAngle
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(algae_aim_proportional())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}