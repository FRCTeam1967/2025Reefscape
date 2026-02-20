// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    Optional<Alliance> ally = DriverStation.getAlliance(); 
    final StructPublisher<Pose2d> towerPublisher = NetworkTableInstance.getDefault().getTable("alignment").getStructTopic("tower", Pose2d.struct).publish();  
    Pose2d towerPose = new Pose2d();

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
          towerPose = new Pose2d(15.56, 2.98, new Rotation2d(0));
          //DogLog.log("Tower Pose: ", towerPose);
      }
      if (ally.get() == Alliance.Blue) {
          towerPose = new Pose2d(0.84, 4.8, new Rotation2d(Math.PI));
          //DogLog.log("Tower Pose: ", towerPose);
      }
      towerPublisher.set(towerPose);
      DogLog.log("Tower Pose: ", towerPose);  
    }
  }

  public void setFirstVisionPose() {
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

    drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.0,0.0,9999999));

    drivetrain.addVisionMeasurement(
      mt2.pose,
      Utils.fpgaToCurrentTime(mt2.timestampSeconds));
  }

  private boolean disableVision = false;
  // private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public NetworkTable limelightTable, limelightOdometryTable;
  public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

  public Pose3d targetPose = new Pose3d();
  public LimelightResults results = new LimelightResults();

  //Limelight Updating Values
  private double xAlignmentOffset, yAlignmentOffset, zAlignmentOffset, vAlignmentCheck;
  private double xOdometryOffset, yOdometryOffset, zOdometryOffset;
  private ChassisSpeeds alignSpeed;

  private boolean isInRange = false;

  public boolean isVisionDisabled(){
    return disableVision;
  }

  public boolean getInRange() {
    return isInRange;
  }

  public void setInRangeTrue() {
    isInRange = true;
  }

  public void setInRangeFalse() {
    isInRange = false;
  }

  public double getTXAlignmentOffset() {
    return xAlignmentOffset;
  }

  public double getAlignmentCheck() {
    return vAlignmentCheck;
  }


  @Override
  public void periodic() {
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight-front", (drivetrain.getPigeon2().getRotation2d().getDegrees()), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(drivetrain.getPigeon2().getRotation2d().getDegrees()) > 360) {
    //   doRejectUpdate = true;
    // }
    if(mt2 != null){
      if(mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
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