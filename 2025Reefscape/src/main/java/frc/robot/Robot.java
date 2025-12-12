// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final AutoFactory autoFactory;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    var drive = m_robotContainer.drivetrain;

    autoFactory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
    );
  }
  public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("driveToMiddle");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
            driveToMiddle.resetOdometry(),
            driveToMiddle.cmd()
        )
    );

    return routine;
}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
