// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class VisionUpdate extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final StructPublisher<Pose2d> limelightPublisher;

  private long odometryUpdates = 0;
  private long odometryDiscards = 0;
  IntegerPublisher updatePublisher;
  IntegerPublisher discardPublisher;

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
    // Tell Limelight what our current orientation is
    var driveState = drivetrain.getState();
    Rotation2d heading = driveState.Pose.getRotation();
    LimelightHelpers.SetRobotOrientation("limelight-santos", heading.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-santos");
    boolean doRejectUpdate = false;

    // If we don't see any tags, the pose can't be good
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      limelightPublisher.set(mt2.pose);
      updatePublisher.set(++odometryUpdates);
    } else {
      discardPublisher.set(++odometryDiscards);
    }
  }

    // //This method will be called once per scheduler run
    // LimelightHelpers.PoseEstimate poseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-santos");    
    // //original: PoseEstimate poseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-santos");    

    // if (poseEstimator.tagCount >= 2) {
    //   RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    // }

    // RobotContainer.drivetrain.addVisionMeasurement(poseEstimator.pose, poseEstimator.timestampSeconds);

    // limelightPublisher.set(poseEstimator.pose);
}