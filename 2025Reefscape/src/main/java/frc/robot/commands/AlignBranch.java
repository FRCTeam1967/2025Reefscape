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
public class AlignBranch extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private double alignmentOffset = 0.0;
  private SlewRateLimiter xLimiter, yLimiter;
  private double xSpeed;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  
    /** Creates a new OffsetAlign. */
    public AlignBranch(CommandSwerveDrivetrain drivetrain, Vision vision) {
      this.drivetrain = drivetrain;
      this.vision = vision;
      addRequirements(drivetrain);
      addRequirements(vision);
    }
  
    // Called when the command is initially scheduled. 
    /**
     * Sets a 3D positional offset for fiducial tracking on the Limelight camera.
     */
    @Override
    public void initialize() {
      vision.setInRangeFalse();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    /**
     * Moves the robot (left-right) by controlling chassis speeds until aligned <br></br>
     * 1) Checks if the xOffset is in range within 3 degrees of the "center" <br></br>
     * 2) If so, applies ChassisSpeeds object to the drivetrain (direction specificed based on where the robot currently is) <br></br>
     * 3) When the robot is within 3 degrees of the "center," set isInRange to true, calling isFinished then end
     */
    @Override
    public void execute() {
      if (vision.getAlignmentCheck() == 0) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);   
        drivetrain.setControl(request.withSpeeds(chassisSpeeds));
      } else {
        alignmentOffset = vision.getAlignmentOffset();
        if (alignmentOffset >= 1.5){ //3.0
          xSpeed = -Constants.Vision.ALIGNMENT_SPEED;
          chassisSpeeds = new ChassisSpeeds(0, xSpeed, 0.0);
          drivetrain.setControl(request.withSpeeds(chassisSpeeds));
          vision.setInRangeFalse();
        } else if (alignmentOffset < 1.5 && alignmentOffset >= -1.5) { //0.0 //3.0 and -1.0
          vision.setInRangeTrue();
        } else {
          xSpeed = Constants.Vision.ALIGNMENT_SPEED;
          chassisSpeeds = new ChassisSpeeds(0.0, xSpeed, 0.0);
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