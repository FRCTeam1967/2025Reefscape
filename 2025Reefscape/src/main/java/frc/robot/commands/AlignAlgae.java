// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAlgae extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private static ChassisSpeeds alignmentSpeed = new ChassisSpeeds();
  private double alignmentOffset = 0.0;
  private double forwardOffset = 0.0;
  boolean canSeeAlgae = false;

  // Create a simple helper class to we can return two values from a method
  // nicely.
  private static class PIDResult {
    public double speed;
    public boolean inRange;

    public PIDResult(double speed, boolean inRange) {
      this.speed = speed;
      this.inRange = inRange;
    }
  }

  /** Creates a new OffsetAlign. */
  public AlignAlgae(CommandSwerveDrivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    addRequirements(drivetrain);
    addRequirements(vision);
  }

  public static ChassisSpeeds getAlignmentSpeed() {
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

  // Returns a Pair object. The first is the caclulated speed (double), and the
  // second (boolean) indicates whether the
  // robot is considered in range in this dimension.
  private PIDResult calculateAlignSpeed(double offset, double kP, double threshold) {
    double error = offset;
    double alignSpeed = error * kP * Constants.Swerve.SWERVE_MAX_SPEED;
    double clampedSpeed = MathUtil.clamp(alignSpeed, -Constants.Vision.ALIGNMENT_SPEED,
        Constants.Vision.ALIGNMENT_SPEED);
    boolean isInRange = Math.abs(error) < threshold;

    return new PIDResult(clampedSpeed, isInRange);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // boolean canSeeTag = LimelightHelpers.getTV("limelight-front");
    // DogLog.log("AlignBranch/seesTag", canSeeTag);
    
    if (LimelightHelpers.getTV("limelight-front") && LimelightHelpers.getDetectorClass("limelight-front").equals("algae")) {
      canSeeAlgae = true;
    }
    
    LimelightHelpers.getTV("limelight-front");
    DogLog.log("AlignAlgae/seesAlgae", canSeeAlgae);

    if (LimelightHelpers.getDetectorClass("limelight-front").equals("algae")) {
      alignmentOffset = LimelightHelpers.getTX("limelight-front");
      forwardOffset = LimelightHelpers.getTY("limelight-front");
    }

    var resultX = calculateAlignSpeed(forwardOffset, Constants.Vision.ALIGNMENT_X_KP, Constants.Vision.FORWARD_ALIGNMENT_THRESHOLD);
    var resultROT = calculateAlignSpeed(alignmentOffset, Constants.Vision.ALIGNMENT_ROT_KP, Constants.Vision.ALIGNMENT_THRESHOLD); //rot kp -0.1

    DogLog.log("AlignAlgae/offset", alignmentOffset);

    alignmentSpeed = new ChassisSpeeds(resultX.speed, 0.0, resultROT.speed);
    drivetrain.setControl(request.withSpeeds(alignmentSpeed));
    DogLog.log("AlignAlgae/appliedChassisSpeeds", alignmentSpeed);

    if (resultX.inRange && resultROT.inRange) {
        vision.setInRangeTrue();
    } 
    else {
        vision.setInRangeFalse();
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