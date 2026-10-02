// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;

public class AimAlgae extends Command {
  public final CommandSwerveDrivetrain swerve;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric driveAtAngle = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.0) // 0.1 = deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AimAlgae(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public double algae_aim_proportional() {        
      double kP = 0.035;
      double targetingAngularVelocity = 0.0; 
      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
      // your limelight 3 feed, tx should return roughly 31 degrees.

    // for algae
    //   if (LimelightHelpers.getTV("limelight-front")) {
    //       DogLog.log("Visabelle/can see algae?", true);
    //       String detectedClass = LimelightHelpers.getDetectorClass("limelight-front");
          
    //       if (detectedClass != null && detectedClass.equals("algae")) { 
    //           DogLog.log("algae tx", LimelightHelpers.getTX("limelight-front"));
    //           targetingAngularVelocity = (LimelightHelpers.getTX("limelight-front") * kP); 
    //       }
    //   }

    // for apriltags; just to test
      if (LimelightHelpers.getTV("limelight-front")) {
            double tx = LimelightHelpers.getTX("limelight-front");
        //   DogLog.log("Visabelle/can see algae?", true);
        //   String detectedClass = LimelightHelpers.getDetectorClass("limelight-front");
        
        //   if (detectedClass != null && detectedClass.equals("algae")) { 
            DogLog.log("algae tx", tx);
            targetingAngularVelocity = tx * kP * MaxAngularRate;
        //   }
      }

      else {
          DogLog.log("Visabelle/can see algae?", false);
      }

      // convert to actual rad/s for our drive method
      // requested speed = |tx| * 0.035 * MaxAngularRate
      // rotational deadband = 0.1 * MaxAngularRate

      // should move when |tx| * 0.035 * MaxAngularRate > 0.1 * MaxAngularRate
      // 0.035 * |tx| > 0.1
      // |tx| > 2.86

      // invert if the robot turns away from the target instead of toward it
        targetingAngularVelocity *= -1.0;
        targetingAngularVelocity = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, targetingAngularVelocity));

      DogLog.log("Visabelle/target algae angle", targetingAngularVelocity);
      return targetingAngularVelocity;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setControl(driveAtAngle
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(algae_aim_proportional()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(driveAtAngle.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    DogLog.log("Visabelle/AimAlgae end", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (LimelightHelpers.getTV("limelight-front")) {
        if (Math.abs(LimelightHelpers.getTX("limelight-front")) <= 1) {
            return true;
        }
    }
    return false;
  }
}