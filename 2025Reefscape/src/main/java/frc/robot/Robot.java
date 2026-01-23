// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  boolean enableLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    /* if (enableLimelight) {

      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 2) {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        RobotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    } */
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    LimelightHelpers.SetIMUMode("limelight-front", 1);
  }

  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetThrottle("limelight-front", 200);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    LimelightHelpers.SetIMUMode("limelight-front", 2);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    LimelightHelpers.SetIMUMode("limelight-front", 2);
    m_robotContainer.vision.setFirstVisionPose();

  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}