// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class VisionUpdate extends SubsystemBase {
  /** Creates a new VisionUpdate. */

  // visibility dataType name;
  private CommandSwerveDrivetrain drivetrain;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private final StructPublisher<Pose2d> limelightPublisher;

  private long odometryUpdates = 0;
  private long odometryDiscards = 0;
  IntegerPublisher updatePublisher;
  IntegerPublisher discardPublisher;

  DoublePublisher LLtimestamp;
  DoublePublisher fpgaTimestamp;
  DoublePublisher LLtoFPGA;

  
  public VisionUpdate(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    limelightPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("odometryStats");

    updatePublisher = table.getIntegerTopic("acceptedUpdates").publish();
    discardPublisher = table.getIntegerTopic("rejectedUpdates").publish();


    LLtimestamp = table.getDoubleTopic("limelight timestamp").publish();
    fpgaTimestamp = table.getDoubleTopic("fpga timestamp").publish();
    LLtoFPGA = table.getDoubleTopic("LL converted to fpga").publish();

  }

  public void setFirstVisionPose() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

    drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));
    drivetrain.addVisionMeasurement(
      mt2.pose,
      Utils.fpgaToCurrentTime(mt2.timestampSeconds));
  }

  @Override
  public void periodic() {
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight-front", drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(drivetrain.getPigeon2().getRotation2d().getDegrees()) > 360) {
    //   doRejectUpdate = true;
    // }
    
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    
    if(!doRejectUpdate){
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      drivetrain.addVisionMeasurement(
          mt2.pose,
          Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      limelightPublisher.set(mt2.pose);
      updatePublisher.set(++odometryUpdates);
    }
    else {
      discardPublisher.set(++odometryDiscards);
    }

    LLtimestamp.set(mt2.timestampSeconds);
    fpgaTimestamp.set(Utils.getCurrentTimeSeconds());
    LLtoFPGA.set(Utils.fpgaToCurrentTime(mt2.timestampSeconds));

  }
}
