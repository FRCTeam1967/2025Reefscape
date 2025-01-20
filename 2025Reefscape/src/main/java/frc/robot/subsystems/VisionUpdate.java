// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionUpdate extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  private final LimelightTarget_Retro retro = new LimelightTarget_Retro();

  /** Creates a new VisionUpdate. */
  public VisionUpdate() {
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-janky");
    RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Timer.getFPGATimestamp());

    System.out.println(RobotContainer.drivetrain.getState().Pose);
    System.out.println(retro.getRobotPose_FieldSpace());

    m_field.setRobotPose(RobotContainer.vision.getBlueFieldX(),
                        RobotContainer.vision.getBlueFieldY(),
                        new Rotation2d(RobotContainer.vision.getBlueFieldRot()*Math.PI/180)
    );



  /**
   * if (enable) {
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults(ll).targetingResults;
      if (result.valid) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
        if (field.isPoseWithinArea(botpose)) {
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || result.targets_Fiducials.length > 1) {
            drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(confidence, confidence, .01));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }
    }
   */




  }
}
