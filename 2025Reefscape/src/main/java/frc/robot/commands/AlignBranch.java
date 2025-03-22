// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import static java.lang.Math.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignBranch extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private double error = 0.0;
  private SlewRateLimiter xLimiter, yLimiter;
  private double xSpeed;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private static double alignmentSpeed;
  private static boolean isLeft;
  private static boolean useForward;
  private static double threshold; 
  
    /** Creates a new OffsetAlign. */
    public AlignBranch(CommandSwerveDrivetrain drivetrain, Vision vision, boolean isLeft, boolean useForward) {
      this.drivetrain = drivetrain;
      this.vision = vision;      
      addRequirements(drivetrain);
      addRequirements(vision);
    }

    public static double getAlignmentSpeed(){
      return alignmentSpeed;
    }
  
    // Called when the command is initially scheduled. 
    /**
     * Sets a 3D positional offset for fiducial tracking on the Limelight camera.
     */
    @Override
    public void initialize() {
      vision.setInRangeFalse();
    }

    private void setSpeed(double offset, boolean moveX, double target){
      double error = offset - target;

      double kp = 0.017;
      double proportionalOffset = error * kp * -1.0;

      alignmentSpeed = proportionalOffset * Constants.Swerve.SWERVE_MAX_SPEED;
        
      double finalAlignmentSpeed = MathUtil.clamp(alignmentSpeed, -Constants.Vision.ALIGNMENT_SPEED, Constants.Vision.ALIGNMENT_SPEED);

      if (moveX){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(finalAlignmentSpeed, 0.0, 0.0);
      } else {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, finalAlignmentSpeed, 0.0);
      }

      drivetrain.setControl(request.withSpeeds(chassisSpeeds));
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (useForward){
        setSpeed(vision.getZOffset(), false, Constants.Vision.ALIGNMENT_FORWARD_OFFSET);
        threshold = Constants.Vision.FORWARD_ALIGNMENT_THRESHOLD;
      } else {
        
        if (isLeft){
          setSpeed(vision.getAlignmentOffset(), false, Constants.Vision.ALIGNMENT_LEFT_OFFSET);
          threshold = Constants.Vision.ALIGNMENT_THRESHOLD;
        } else {
          setSpeed(vision.getAlignmentOffset(), false, Constants.Vision.ALIGNMENT_RIGHT_OFFSET);
          threshold = Constants.Vision.ALIGNMENT_THRESHOLD;
        }
  
        if (Math.abs(error) <= threshold){
          vision.setInRangeTrue();
        } else{
          vision.setInRangeFalse();
        }
      }

      // OLD VISION JUST IN CASE
      // if (vision.getAlignmentCheck() == 0) {
      //   ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);   
      //   drivetrain.setControl(request.withSpeeds(chassisSpeeds));
      // } else {
      //   alignmentOffset = vision.getAlignmentOffset();
      //   if (alignmentOffset >= 1.0){ //3.0
      //     xSpeed = -Constants.Vision.ALIGNMENT_SPEED;
      //     chassisSpeeds = new ChassisSpeeds(0, xSpeed, 0.0);
      //     drivetrain.setControl(request.withSpeeds(chassisSpeeds));
      //     vision.setInRangeFalse();
      //   } else if (alignmentOffset < 1.0 && alignmentOffset >= -1.0) { //0.0 //3.0 and -1.0
      //     vision.setInRangeTrue();
      //   } else {
      //     xSpeed = Constants.Vision.ALIGNMENT_SPEED;
      //     chassisSpeeds = new ChassisSpeeds(0.0, xSpeed, 0.0);
      //     drivetrain.setControl(request.withSpeeds(chassisSpeeds));
      //     vision.setInRangeFalse();
      //   }
      // }
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
