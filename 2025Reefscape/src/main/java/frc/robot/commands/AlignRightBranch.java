// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRightBranch extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private SlewRateLimiter xLimiter, yLimiter;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  /** Creates a new OffsetAlign. */
  public AlignRightBranch(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    addRequirements(drivetrain, vision);
  }


  /**
   * Cube the input from the joystick for a smooth movement (exponential vs linear acceleration)
   */
  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling){
    input = Math.pow(input, 3);
    input = Math.abs(input)> deadband ? input : 0;
    input *= speedScaling;

    return input;
  }

  public boolean getIsInRange(){
    if (vision.getOffset() < 5.0 && vision.getOffset() >= -2.0) {
      return true;
    }
    return false;
  }

  // Called when the command is initially scheduled. 
  /**
   * Sets a 3D positional offset for fiducial tracking on the Limelight camera.
   *
   */
  @Override
  public void initialize() {
    LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.1513, 0.0); //0.2275-0.076 //0.2286, left, -.0658890
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Check if the vision system detects an offset greater than or equal to 5.0
   * 1)Move left based on the chasis speed and line 56 sends the speed command to the drivetrain
   * 2)Check if the vision offset is between -2.0 and 5.0 
   */
  @Override
  public void execute() {
    if (vision.getOffset() >= 5.0){
      double xSpeed = cleanAndScaleInput(0.0, -0.6, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, xSpeed, 0.0);
      
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

    } else{
      double xSpeed = cleanAndScaleInput(0.0, 0.6, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, xSpeed, 0.0);
      
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);   
    RobotContainer.drivetrain.setControl(request.withSpeeds(chassisSpeeds));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getVisionAbility() || getIsInRange();
  }
}
