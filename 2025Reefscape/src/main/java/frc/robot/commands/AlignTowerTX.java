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
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerTX extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionUpdate vision;
  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
  private static ChassisSpeeds alignmentSpeed = new ChassisSpeeds();
  private double alignmentOffset = 0.0; //CHANGE WITH TESTING
  private boolean isLeft;
  private boolean useForward;

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
  public AlignTowerTX(CommandSwerveDrivetrain drivetrain, VisionUpdate vision, boolean isLeft, boolean useForward) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.isLeft = isLeft;
    this.useForward = useForward;
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
  private PIDResult calculateAlignSpeed(double offset, double targetOffset, double kP, double threshold) {
    double error = offset - targetOffset;
    double alignSpeed = error * kP * Constants.Swerve.SWERVE_MAX_SPEED;
    double clampedSpeed = MathUtil.clamp(alignSpeed, -Constants.Vision.ALIGNMENT_SPEED,
        Constants.Vision.ALIGNMENT_SPEED);
    boolean isInRange = Math.abs(error) < threshold;

    return new PIDResult(clampedSpeed, isInRange);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean canSeeTag = vision.getAlignmentCheck() != 0.0; // Also poor that this is a double, but that's what Limelight does...
    DogLog.log("AlignTower/seesTag", canSeeTag);

    // Calculate forward speed (X), if we're being asked to
    PIDResult resultX;
    // if (useForward) {
    //   resultX = calculateAlignSpeed(vision.getZOffset(), Constants.Vision.ALIGNMENT_FORWARD_OFFSET,
    //       Constants.Vision.ALIGNMENT_X_KP, Constants.Vision.FORWARD_ALIGNMENT_THRESHOLD);
    // } else {
    //   // If we're not moving forward, consider ourselves in range with 0 speed
      resultX = new PIDResult(0.0, true);
    //}
    alignmentOffset = vision.getTXAlignmentOffset();
    // Calculate side-side speed
    var resultY = calculateAlignSpeed(alignmentOffset,
        isLeft ? Constants.Vision.ALIGNMENT_LEFT_OFFSET : Constants.Vision.ALIGNMENT_RIGHT_OFFSET,
        Constants.Vision.ALIGNMENT_Y_KP, Constants.Vision.ALIGNMENT_THRESHOLD);

    DogLog.log("AlignTower/offset", alignmentOffset);

    alignmentSpeed = new ChassisSpeeds(resultX.speed, resultY.speed, 0.0);
    drivetrain.setControl(request.withSpeeds(alignmentSpeed));
    DogLog.log("AlignTower/appliedChassisSpeeds", alignmentSpeed);

    if (resultX.inRange && resultY.inRange) {
      vision.setInRangeTrue();
    } else {
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