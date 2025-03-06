// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionUpdate extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightTarget_Retro retro = new LimelightTarget_Retro(); // MS: Not sure what this is. Delete?
  private final StructPublisher<Pose2d> limelightPublisher;

  // MS: It might be useful to collect how many times we applied and didn't apply an odometry update
  private long odometryUpdates = 0;
  private long odometryDiscards = 0;
  IntegerPublisher updatePublisher;
  IntegerPublisher discardPublisher;

  // MS: This class should take the limelight name in the constructor. If we had multiple limelights that could see tags, we could
  // construct multiple instances of this class, and they each could contribute to odometry! Also the limelight name should be a 
  // constant (that's passed in here).
  /** Creates a new VisionUpdate. */
  public VisionUpdate(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    limelightPublisher = NetworkTableInstance.getDefault().getTable("limelight-santos").getStructTopic("Limelight Pose", Pose2d.struct).publish();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("odometryStats");
    updatePublisher = table.getIntegerTopic("acceptedUpdates").publish();
    discardPublisher = table.getIntegerTopic("rejectedUpdates").publish();
  }

  @Override
  public void periodic() {
    // MS: This is the MegaTag (1) implementation, and has some issues when we only see one tag. I don't think it was always this way
    // but their current sample code only accepts a vision measurement if it can see at least 2 tags, but we're only ever going to
    // see one. I think we should probalby use the MegaTag2 implementation.
    //This method will be called once per scheduler run
    LimelightHelpers.PoseEstimate poseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-santos");    
    //original: PoseEstimate poseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-santos");    

    if (poseEstimator.tagCount >= 2) {
      RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    }

    // MS: We captured the drivetrain in the constructor. Why go back to RobotContainer to get it here?
    // MS: Also, this is adding a vision measurement even if the camera can't see anything, in which case the estimate is at best stale if 
    // not just outright wrong.
    RobotContainer.drivetrain.addVisionMeasurement(poseEstimator.pose, poseEstimator.timestampSeconds);

    limelightPublisher.set(poseEstimator.pose);
  }

  // MS: Here's a replacement periodic for your consideration. Basically stolen from the sample code, but I removed the angular velocity limit
  // since I don't see a good way to get that out of CTRE Swerve.
  public void alt_periodic() {
    // Tell Limelight what our current orientation is
    LimelightHelpers.SetRobotOrientation("limelight-santos", drivetrain.getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-santos");
    boolean doRejectUpdate = false;

    // If we don't see any tags, the pose can't be good
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      limelightPublisher.set(mt2.pose);
      updatePublisher.set(++odometryUpdates);
    } else {
      discardPublisher.set(++odometryDiscards);
    }
  }

}
