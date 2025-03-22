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
public class ZAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private double error = 0.0;
  private SlewRateLimiter xLimiter, yLimiter;
  private double xSpeed;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.0, 0.0);
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private static double alignmentSpeed;
  private static boolean isLeft;
  
    /** Creates a new OffsetAlign. */
    public ZAlign(CommandSwerveDrivetrain drivetrain, Vision vision) {
      this.drivetrain = drivetrain;
      this.vision = vision;
      this.isLeft = isLeft;
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
      //LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0);

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
      error = vision.getZOffset()-Constants.Vision.ALIGNMENT_LEFT_OFFSET;
        
      double kp = 0.017;
      double proportionalOffset = error * kp * -1.0;

      alignmentSpeed = proportionalOffset * Constants.Swerve.SWERVE_MAX_SPEED;
        
      double finalAlignmentSpeed = MathUtil.clamp(alignmentSpeed, -Constants.Vision.ALIGNMENT_SPEED, Constants.Vision.ALIGNMENT_SPEED);

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(finalAlignmentSpeed, 0.0, 0.0);
      drivetrain.setControl(request.withSpeeds(chassisSpeeds));

      if (Math.abs(error) <= Constants.Vision.ALIGNMENT_THRESHOLD){
        vision.setInRangeTrue();
      } else{
        vision.setInRangeFalse();
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
