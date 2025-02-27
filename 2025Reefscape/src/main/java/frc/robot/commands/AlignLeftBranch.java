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

public class AlignLeftBranch extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private SlewRateLimiter xLimiter, yLimiter;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public AlignLeftBranch(CommandSwerveDrivetrain drivetrain, Vision vision) {
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

  /** @return whether limelight is in range */
  public boolean getIsInRange(){
    if (vision.getOffset() < 5.0 && vision.getOffset() >= -2.0) {
      return true;
    }
    return false;
  }

  // Called when the command is initially scheduled.
  /**
   * Sets fiducial 3D offset to where the limelight is supposed to align (fiducial offset -- an offset based on the april tag, measured in meters) <br></br>
   * 1) Sets x, y and z offsets (in meters) -- y is left/right <br></br>
   * 2) This offset accounts for the position of the limelight (which is slightly offset to the right) <br></br>
   */
  @Override
  public void initialize() {
    LimelightHelpers.setFiducial3DOffset("limelight", 0.0, -0.1556, 0.0); //-0.181 //-0.18129 //-0.10509 //-0.0889-0.0762
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Moves the robot (left-right) by controlling chassis speeds until aligned <br></br>
   * 1) Checks if the xOffset is in range within 5 degrees of the "center" <br></br>
   * 2) If so, applies ChassisSpeeds object to the drivetrain (direction specificed based on where the robot currently is) <br></br>
   * 3) When the robot is within 5 degrees of the "center," apply 0 ChassisSpeeds (stop movement)
   */
  @Override
  public void execute() {
    if (vision.getOffset() >= 3.0){
      double xSpeed = cleanAndScaleInput(0.0, -0.45, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, xSpeed, 0.0);

      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

    } else if (vision.getOffset() < 3.0 && vision.getOffset() >= -1.0) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);

      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

    } else{
      double xSpeed = cleanAndScaleInput(0.0, 0.45, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
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

  // Returns true when the command should eSSnd.
  @Override
  public boolean isFinished() {
    return vision.getVisionAbility();
  }
}