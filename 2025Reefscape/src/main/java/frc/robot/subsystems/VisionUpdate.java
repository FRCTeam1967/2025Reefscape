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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionUpdate extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Field2d m_field = new Field2d();
  private final LimelightTarget_Retro retro = new LimelightTarget_Retro();

  /** Creates a new VisionUpdate. */
  public VisionUpdate(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    PoseEstimate poseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-janky");
    
    if (poseEstimator.tagCount >= 2) {
      RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      RobotContainer.drivetrain.addVisionMeasurement(poseEstimator.pose, poseEstimator.timestampSeconds);
    } else {
      RobotContainer.drivetrain.addVisionMeasurement(poseEstimator.pose, poseEstimator.timestampSeconds);
    }

    System.out.println(RobotContainer.drivetrain.getState().Pose);
    System.out.println(retro.getRobotPose_FieldSpace());

    m_field.setRobotPose(RobotContainer.drivetrain.getState().Pose);

    /*m_field.setRobotPose(RobotContainer.vision.getBlueFieldX(),
                        RobotContainer.vision.getBlueFieldY(),
                        RobotContainer.vision.getBlueFieldRot().toRotation2d());*/
  }
}
