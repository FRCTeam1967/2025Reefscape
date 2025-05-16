// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;

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
  }

  @Override
  public void disabledPeriodic() {
    //m_robotContainer.vision.setFiducials();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.elevator.setSafe();
    m_robotContainer.coralPivot.maintainPosition();
    m_robotContainer.algaeMechanism.maintainPosition();
    //m_robotContainer.algaeMechanism.setReltoAbs();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.elevator.setSafe();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    SignalLogger.start();

  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.drivetrain.updateOdometryPoseEstimator();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.elevator.setSafe();     //TODO: REMOVE FROM TELEOP AFTER TESTING

    m_robotContainer.algaeMechanism.setReltoAbs();     //TODO: REMOVE FROM TELEOP AFTER TESTING


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
   //var rotationFlipped = m_robotContainer.drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg);
   // m_robotContainer.drivetrain.resetRotation(rotationFlipped);

    SignalLogger.start();
    m_robotContainer.elevator.setSafe();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
