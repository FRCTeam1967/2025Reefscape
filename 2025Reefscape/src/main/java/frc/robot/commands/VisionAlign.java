// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class VisionAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private SlewRateLimiter xLimiter, yLimiter;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public VisionAlign(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;



    addRequirements(drivetrain, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling){
    input = Math.pow(input, 3);
    input = Math.abs(input)> deadband ? input : 0;
    input *= speedScaling;

    return input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setFiducial3DOffset("limelight-janky", 0.0, 0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getOffset() >= 2.5){
      double xSpeed = cleanAndScaleInput(0.0, 0.45, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, xSpeed, 0.0);
      
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

    } else if (vision.getOffset() < 2.5 && vision.getOffset() >= -1.0) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);
      
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

    } else{
      double xSpeed = cleanAndScaleInput(0.0, -0.45, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, xSpeed, 0.0);
      
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getIsInRange();
  }
}