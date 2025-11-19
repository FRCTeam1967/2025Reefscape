// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import dev.doglog.DogLog;

import java.lang.reflect.Array;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.Units.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.*;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.Math.Matrix;

public class VisionUpdate extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final StructPublisher<Pose2d> limelightPublisher;

  private long odometryUpdates = 0;
  private long odometryDiscards = 0;
  IntegerPublisher updatePublisher;
  IntegerPublisher discardPublisher;
  String camTitle;
  LimelightResults latestVisionResult;
  LimelightTarget_Detector detector;
  BooleanSupplier isRed;
  
  private static final List<Integer> BLUE_SIDE_TAG_IDS = List.of(19, 20, 21, 22, 17, 18);
  private static final List<Integer> RED_SIDE_TAG_IDS = List.of(6, 7, 8, 9, 10, 11);

  private double maximumAllowedDistance = 15.0; // meters, beyond which readings are dropped, from 3501
  
  /** Creates a new VisionUpdate. */
  public VisionUpdate(CommandSwerveDrivetrain drivetrain, SwerveDrivePoseEstimator m_poseEstimator, BooleanSupplier isRed) {
    this.drivetrain = drivetrain;
    this.m_poseEstimator = m_poseEstimator;
    this.isRed = isRed;

    camTitle = "limelight-santos";

    limelightPublisher = NetworkTableInstance.getDefault().getTable("limelight-santos").getStructTopic("Limelight Pose", Pose2d.struct).publish();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("odometryStats");

    updatePublisher = table.getIntegerTopic("acceptedUpdates").publish();
    discardPublisher = table.getIntegerTopic("rejectedUpdates").publish();

    LimelightResults latestVisionResult = new LimelightResults();

    LimelightTarget_Detector detector = new LimelightTarget_Detector();
  }
  
  @Override
  public void periodic() {
    // Tell Limelight what our current orientation is
    var driveState = drivetrain.getState();
    Pose2d robotPose = driveState.Pose;
    Rotation2d heading = robotPose.getRotation();

    LimelightHelpers.SetRobotOrientation("limelight-santos", heading.getDegrees(), 0, 0, 0, 0, 0);
    //LimelightHelpers.SetRobotOrientation("limelight-santos", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    Rotation2d pigeonYaw = drivetrain.getPigeon2().getRotation2d();
    Rotation2d rawHeading = driveState.RawHeading;

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-santos");
    boolean doRejectUpdate = false;

    // m_poseEstimator.update(
    //   drivetrain.getPigeon2().getRotation2d(),
    //   new SwerveModulePosition[] {
    //     drivetrain.getModules(m_frontLeft).getPosition(),
        
    //     m_frontLeft.getPosition(),
    //     m_frontRight.getPosition(),
    //     m_backLeft.getPosition(),
    //     m_backRight.getPosition()
    //   });

    DogLog.log("VisionUpdate/drivetrainHeading", heading);
    DogLog.log("VisionUpdate/pigeonYaw", pigeonYaw);
    DogLog.log("VisionUpdate/tagCount", mt2 != null ? mt2.tagCount : 0);
    DogLog.log("VisionUpdate/drivetrainRawHeading", rawHeading);
    DogLog.log("VisionUpdate/drivetrainPose", robotPose);

    // If we don't see any tags, the pose can't be good
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,0.7,0)); //.7, .7, 9999999
      drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      DogLog.log("VisionUpdate/mt2Pose", mt2.pose);
      limelightPublisher.set(mt2.pose);
      updatePublisher.set(++odometryUpdates);
    } else {
      discardPublisher.set(++odometryDiscards);
    }

    DogLog.log("VisionUpdate/acceptedUpdate", !doRejectUpdate);
  }

  //helper filtering method
  public void addFilteredPose() {
    SwerveDrivePoseEstimator selectedEstimator = m_poseEstimator;

    if (latestVisionResult == null || latestVisionResult.botpose_tagcount == 0) {
      DogLog.log("Vision/" + camTitle + "/HasTargets", false);
      return;
    }
    
    DogLog.log("Vision/" + camTitle + "/HasTargets", true);

    double averageDistance = latestVisionResult.botpose_avgdist;

    double minDistance = 
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    // Filter tags to only those on the active side
    List<PhotonTrackedTarget> validTags =
        latestVisionResult.getTargets().stream()
            .filter(t -> isTagOnActiveSide(t.getFiducialId()))
            .filter(t -> isNotChopped(t.getYaw()))
            .collect(Collectors.toList());

    DogLog.log("Vision/" + camTitle + "/numValidTags", validTags.size());

    for (PhotonTrackedTarget tag : validTags) {
      DogLog.log("Vision/" + camTitle + "/Area", tag.getArea());
      DogLog.log("Vision/" + camTitle + "/Yaw", tag.getYaw());
    }

    DogLog.log("Vision/" + camTitle + "/allTagIds", allTagIds);

    if (validTags.isEmpty()) {
      DogLog.log("Vision/" + camTitle + "/ValidTags", false);
      return;
    }
    DogLog.log("Vision/" + camTitle + "/ValidTags", true);

    // Log all tags that haven't been thrown out
    int tagCount = validTags.size();

    // Compute effective metrics for solution
    // Use camera→target distance from PV (avoids odometry dependence)

    // nothing to do if rejected based on the minDistance or if no min dist has been found
    if (Double.isNaN(minDistance) || minDistance > maximumAllowedDistance) {
      DogLog.log("Vision/" + camTitle + "/ThrownOutDistance", true);
      return;
    }
    DogLog.log("Vision/" + camTitle + "/ThrownOutDistance", false);

    // find the current speed
    double currentSpeed =
        Math.hypot(
            swerveDrive.getRobotSpeeds().vxMetersPerSecond,
            swerveDrive.getRobotSpeeds().vyMetersPerSecond);
    DogLog.log("Vision/" + camTitle + "/VisionDrivebaseSpeed", currentSpeed);

    double nX =
        computeNoiseXY(
            baseNoiseX,
            distanceExponentialCoefficientX,
            distanceExponentialBaseX,
            angleCoefficientX,
            speedCoefficientX,
            averageDistance,
            currentSpeed,
            tagCount);
    double nY =
        computeNoiseXY(
            baseNoiseY,
            distanceExponentialCoefficientY,
            distanceExponentialBaseY,
            angleCoefficientY,
            speedCoefficientY,
            averageDistance,
            currentSpeed,
            tagCount);
    double nTH =
        computeNoiseHeading(
            baseNoiseTheta,
            distanceCoefficientTheta,
            angleCoefficientTheta,
            speedCoefficientTheta,
            averageDistance,
            currentSpeed,
            tagCount);

    DogLog.log("Vision/" + camTitle + "/speed", currentSpeed);
    DogLog.log("Vision/" + camTitle + "/nX", nX);
    DogLog.log("Vision/" + camTitle + "/nY", nY);
    DogLog.log("Vision/" + camTitle + "/nTH", nTH);
    DogLog.log("Vision/" + camTitle + "/Pose", measuredPose);
    DogLog.log("Vision/" + camTitle + "/averageDistance", averageDistance);

    Matrix<N3, N1> noiseVector = VecBuilder.fill(nX, nY, nTH);
    // Process locally (no cross-camera comparison)
    processPoseEstimate(
        measuredPose,
        averageDistance,
        currentSpeed,
        tagCount,
        latestVisionResult.getTimestampSeconds(),
        noiseVector);
  }

  /** check if valid */
  private boolean isValid(int fiducial, LimelightResults results) {
    return isTagOnActiveSide(fiducial) && isTagNotSkew(results);
  }

  /** check if the tags seen are on active alliance side */
  private boolean isTagOnActiveSide(int tagId) {
    return isRed.getAsBoolean()
        ? RED_SIDE_TAG_IDS.contains(tagId)
        : BLUE_SIDE_TAG_IDS.contains(tagId);
  }

  /** get all tags */
  private int[] getAllTags(LimelightResults result){
    int[] tags = new int[(int) result.botpose_tagcount];
    for (int i = 0; i < result.botpose_tagcount; i++) {
      tags[i] = (int) result.targets_Fiducials[i].fiducialID;
    }

    return tags;
  }

  /** make sure yaw isn't too skewed */
  private boolean isTagNotSkew(LimelightResults result){
    double radians = Math.toRadians(60);
    return (result.getBotPose3d_wpiBlue().getRotation().getAngle() > radians);
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