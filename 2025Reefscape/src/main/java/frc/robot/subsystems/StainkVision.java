// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import frc.robot.LimelightHelpers.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * VisionSystem fuses multi-tag AprilTag measurements from PhotonVision with swerve odometry by
 * dynamically computing measurement noise based on tag count, distance, viewing angle, and robot
 * speed.
 */
public class StainkVision extends SubsystemBase {
  // // static member of AnthonyVision that contains array of all existing AnthonyVision systems
  // private static AnthonyVision[] systemList =
  //     new AnthonyVision[Constants.Vision.Cameras.values().length];

  // // Data type is "Cameras", an enum defined in Constants.java with only two options (left, right)
  // private final Constants.Vision.Cameras cameraId;

  private String camTitle;
  // Reef tag IDs for each side of the field
  private static final List<Integer> BLUE_SIDE_TAG_IDS = List.of(19, 20, 21, 22, 17, 18);
  private static final List<Integer> RED_SIDE_TAG_IDS = List.of(6, 7, 8, 9, 10, 11);

  // Noise parameters
  private double calibrationFactor = 1.0; // constant multiplier to everything
  private double baseNoiseX = 0.0008; // meters
  private double baseNoiseY = 0.0008;
  private double baseNoiseTheta = 0.5; // radians

  // private double distanceCoefficientX = 0.06;
  // private double distanceCoefficientY = 0.06;

  private double distanceExponentialCoefficientX = 0.00046074;
  private double distanceExponentialBaseX = 2.97294;
  private double distanceExponentialCoefficientY = 0.00046074;
  private double distanceExponentialBaseY = 2.97294;

  private double distanceCoefficientTheta = 0.9;

  private double angleCoefficientX = 0.5; // noise growth per radian of viewing angle
  private double angleCoefficientY = 0.5;
  private double angleCoefficientTheta = 0.5;

  private double speedCoefficientX = 0.5; // noise growth per fraction of max speed
  private double speedCoefficientY = 0.5;
  private double speedCoefficientTheta = 0.5;

  // Maximums for normalization
  private double maximumRobotSpeed = 5; // meters per second
  private double maximumAllowedDistance = 15.0; // meters, beyond which readings are dropped

  // PhotonVision and odometry references
  private final SwerveDrivePoseEstimator poseEstimator; // MULTI_TAG_PNP_ON_COPROCESSOR
  private LimelightResults latestVisionResult;
  private final BooleanSupplier isRedSide;
  private CommandSwerveDrivetrain drivetrain;
  // private final AprilTagFieldLayout fieldLayout;

  public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

  public StainkVision(String camTitle, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.camTitle = camTitle;
    // Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraId);  (there os cam)

    // Initialize field layout
    // this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Initialize pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
        drivetrain.getKinematics(), 
        drivetrain.getPigeon2().getRotation2d(), 
        new SwerveModulePosition[] {
            drivetrain.getModule(0).getPosition(true),
            drivetrain.getModule(1).getPosition(true),
            drivetrain.getModule(2).getPosition(true),
            drivetrain.getModule(3).getPosition(true),
        },
        new Pose2d(0.0, 0.0, new Rotation2d())
    );

    latestVisionResult = null;
  }

  // Get instance of AnthonyVision
  public static StainkVision getInstance(
      Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    int idx = cameraId.ordinal();
    if (systemList[idx] == null) {
      systemList[idx] = new StainkVision(cameraId, isRedSide);
    }
    return systemList[idx];
  }

  @Override
  public void periodic() {
    // Initialize swerve drive if not already done
    if (swerveDrive == null) {
      swerveDrive = SwerveSubsystem.getInstance();
    }

    // Check camera connection
    boolean cameraConnected = photonCamera.isConnected();

    // If the current camera isn't connected, there's nothing to do here
    if (!cameraConnected) {
      DogLog.log("Vision/" + camTitle + "/CameraConnected", false);
      return;
    }

    DogLog.log("Vision/" + camTitle + "/CameraConnected", true);
    // Get all unread results
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

    // Go through all results (if there are any) and update the latest result with the last
    for (var result : results) {
      latestVisionResult = result;
    }
  }

  public void addFilteredPose() {
    PhotonPoseEstimator selectedEstimator = poseEstimator;
    if (latestVisionResult == null || !latestVisionResult.hasTargets()) {
      DogLog.log("Vision/" + camTitle + "/HasTargets", false);
      return;
    }
    DogLog.log("Vision/" + camTitle + "/HasTargets", true);

    double averageDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);

    double minDistance = //can maybe use distToCamera field from LimelightHelpers.RawFiducial and loop through to find the min distance
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

    // Log all detected tags for debugging
    String allTagIds =
        latestVisionResult.getTargets().stream()
            .map(t -> Integer.toString(t.getFiducialId()))
            .collect(Collectors.joining(","));

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

    // Get the pose from PhotonVision
    Optional<EstimatedRobotPose> maybePose = selectedEstimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      DogLog.log("Vision/" + camTitle + "/AvailablePose", false);
      return;
    }
    DogLog.log("Vision/" + camTitle + "/AvailablePose", true);

    EstimatedRobotPose estimatedPose = maybePose.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();

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

  //TODO: logs
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

  /** Final processing and addition of pose estimate to odometry. */
  private void processPoseEstimate(
      Pose2d measuredPose,
      double averageDistance,
      double currentSpeed,
      int tagCount,
      double timestamp,
      Matrix<N3, N1> noiseVector) {
    // Choose timestamp: use vision timestamp unless it differs too much from FPGA
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDifference = Math.abs(timestamp - fpgaTimestamp);
    double chosenTimestamp = (timestampDifference > 0.5) ? fpgaTimestamp - 0.03 : timestamp;

    // Build the noise vector and add the vision measurement

    swerveDrive.addVisionMeasurement(measuredPose, chosenTimestamp, noiseVector);
  }

  private boolean isTagOnActiveSide(int tagId) {
    return isRedSide.getAsBoolean()
        ? RED_SIDE_TAG_IDS.contains(tagId)
        : BLUE_SIDE_TAG_IDS.contains(tagId);
  }\

  private boolean isNotChopped(double yaw) {
    return (Math.abs(yaw) < 60d);
  }

  private double computeNoiseXY(
      double baseNoise,
      double distanceExponentialCoefficient,
      double distanceExponentialBase,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (diminishing returns; cap at 4)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1.0 / Math.sqrt(effectiveTags);

    // Distance term (keep as d^2)
    // double distanceFactor = baseNoise +
    // distanceExponentialCoefficient*Math.pow(distanceExponentialBase, distance);

    double distanceFactor =
        (distance < (17.548 + 0.67))
            ? Math.min(
                baseNoise
                    + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance),
                1.167)
            : (baseNoise
                + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance));

    // Speed term (quadratic, saturated)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1.0 + speedCoefficient * (vNorm * vNorm);
    // TODO: Logs

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }

  private double computeNoiseHeading(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (diminishing returns; cap at 4)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1.0 / Math.sqrt(effectiveTags);

    // Distance term (keep as d^2)
    double distanceFactor = baseNoise + distanceCoefficient * distance * distance;

    // Speed term (quadratic, saturated)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1.0 + speedCoefficient * (vNorm * vNorm);

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }
}