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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private double alignmentOffset = 0.0;
  private double zSpeed;
  private boolean isInRange = false;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  /** Creates a new ZAlign. */
  public ZAlign(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  /**
   * Sets fiducial 3D offset to where the limelight is supposed to align (fiducial offset -- an offset based on the april tag, measured in meters) <br></br>
   * 1) Sets x, y and z offsets (in meters) -- y is left/right <br></br>
   * 2) This offset accounts for the position of the limelight (which is slightly offset to the right) <br></br>
   */
  @Override
  public void initialize() {
    vision.setInRangeFalse();
  }


  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Moves the robot (left-right) by controlling chassis speeds until aligned <br></br>
   * 1) Checks if the y? z? Offset is in range within 5 degrees of the "center" <br></br>
   * 2) If so, applies ChassisSpeeds object to the drivetrain (direction specificed based on where the robot currently is) <br></br>
   * 3) When the robot is within 5 degrees of the "center," apply 0 ChassisSpeeds (stop movement)
   */
  @Override
  public void execute() {
    if (vision.getAlignmentCheck() == 0) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);   
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));
    } else {
      alignmentOffset = vision.getZOffsets();
      // MS: These offset values really ought to be in Constants so it's easier to tune them. Maybe even have them be on the dashboard and read so that
      // tuning doesn't require redeploying
      if (alignmentOffset >= 3.0){
        zSpeed = Constants.Vision.ALIGNMENT_SPEED;
        chassisSpeeds = new ChassisSpeeds(zSpeed, 0.0, 0.0);
        drivetrain.setControl(request.withSpeeds(chassisSpeeds));
        vision.setInRangeFalse();
      } else if (alignmentOffset < 3.0 && alignmentOffset >= 0.0) {
        vision.setInRangeTrue();
      } else {
        zSpeed = -Constants.Vision.ALIGNMENT_SPEED;
        chassisSpeeds = new ChassisSpeeds(zSpeed, 0.0, 0.0);
        drivetrain.setControl(request.withSpeeds(chassisSpeeds));
        vision.setInRangeFalse();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);   
    drivetrain.setControl(request.withSpeeds(chassisSpeeds));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getInRange() || vision.isVisionDisabled();
  }
}
