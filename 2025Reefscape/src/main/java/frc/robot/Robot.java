// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import dev.doglog.DogLog;

public class Robot extends TimedRobot {
  public final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  boolean enableLimelight = false;
  
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  private final StructPublisher<Pose2d> choreoPublisher;
  //private final NetworkTableListener autoPublisher;

  public Robot() {
    m_robotContainer = new RobotContainer();

    autoFactory = new AutoFactory(
            m_robotContainer.drivetrain::getPose, // A function that returns the current robot pose
            m_robotContainer.drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            m_robotContainer.drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_robotContainer.drivetrain // The drive subsystem
    );
    autoChooser = new AutoChooser();

    autoChooser.addRoutine("Test", this::Test);
    autoChooser.addRoutine("TestTrench2", this::TestTrench2);
    autoChooser.addRoutine("AngleRedTrench", this::AngleRedTrench);
    autoChooser.addRoutine("TowerDepot", this::TowerDepot);
    autoChooser.addRoutine("TowerOutpost", this::TowerOutpost);
    autoChooser.addRoutine("TowerWrong", this::TowerWrong);
    autoChooser.addRoutine("TowerWronger", this::TowerWronger);
    autoChooser.addRoutine("TowerMcQueen", this::TowerMcQueen);

    //autoPublisher = NetworkTableInstance.getDefault().getTable("selected_auto");

    matchTab.add("auto chooser lol", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    //RobotModeTriggers.disabled().whileTrue(autoChooser.selectedCommandScheduler()); 
    
    choreoPublisher = NetworkTableInstance.getDefault().getTable("limelight-front").getStructTopic("Limelight Pose", Pose2d.struct).publish();
  }

  //CHOREO AUTO
  private AutoRoutine Test() {
      AutoRoutine routine = autoFactory.newRoutine("test");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("Test");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //logging
              new InstantCommand(() -> DogLog.log("initial pose from choreo", test_path.getInitialPose().get())),
              new InstantCommand(() -> DogLog.log("final pose from choreo", test_path.getFinalPose().get())),

              new InstantCommand(() -> DogLog.log("gyro BEFORE conditional", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees())),
              new InstantCommand(() -> DogLog.log("limelight yaw BEFORE resetOdometry", Units.radiansToDegrees(LimelightHelpers.getBotPose3d("limelight-front").getRotation().getZ()))),
              new InstantCommand(() -> DogLog.log("should flip?", ChoreoAllianceFlipUtil.shouldFlip())),

              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
              new InstantCommand(() -> DogLog.log("gyro position after 'setting' it", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees())),

              new InstantCommand(() -> DogLog.log("limelight yaw AFTER resetOdometry", Units.radiansToDegrees(LimelightHelpers.getBotPose3d("limelight-front").getRotation().getZ()))),
                
              //step four: run the path!
              test_path.cmd()          
          )
      );

      return routine;
  }

  private AutoRoutine TestTrench2() {
      AutoRoutine routine = autoFactory.newRoutine("TestTrench2");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("TestTrench2");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd()                  
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());


      return routine;
  }

  private AutoRoutine AngleRedTrench() {
      AutoRoutine routine = autoFactory.newRoutine("AngleRedTrench");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("AngleRedTrench");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd()      
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());

      return routine;
  }

  private AutoRoutine TowerDepot() {
      AutoRoutine routine = autoFactory.newRoutine("TowerDepot");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("TowerDepot");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd()      
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());

      return routine;
  }

  private AutoRoutine TowerOutpost() {
      AutoRoutine routine = autoFactory.newRoutine("TowerOutpost");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("TowerOutpost");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd()      
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());

      return routine;
  }

  private AutoRoutine TowerWrong() {
      AutoRoutine routine = autoFactory.newRoutine("TowerWrong");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("TowerWrong");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd(),
              new WaitCommand(2.0),
              new AlignTowerPose(m_robotContainer.drivetrain)
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());

      return routine;
  }

    private AutoRoutine TowerWronger() {
      AutoRoutine routine = autoFactory.newRoutine("TowerWronger");
      // Load the routine's trajectories
      // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
      AutoTrajectory test_path = routine.trajectory("TowerWronger");

      // When the routine begins, reset odometry and start the first trajectory (1)
      routine.active().onTrue(
          Commands.sequence(
              //step one: set gyro to starting heading (flips for alliance)
              new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

              //step two: reset odometry to starting pose
              test_path.resetOdometry(),

              //step three: set LL heading to gyro (aka starting) heading
              new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
                
              //step four: run the path!
              test_path.cmd(),
              new AlignTowerPose(m_robotContainer.drivetrain)
          )
      );

      //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());
      return routine;
  }

  private AutoRoutine TowerMcQueen() {
    AutoRoutine routine = autoFactory.newRoutine("TowerMcQueen");
    // Load the routine's trajectories
    // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
    AutoTrajectory test_path = routine.trajectory("TowerMcQueen");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(
        Commands.sequence(
            //step one: set gyro to starting heading (flips for alliance)
            new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees())),

            //step two: reset odometry to starting pose
            test_path.resetOdometry(),

            //step three: set LL heading to gyro (aka starting) heading
            new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0)),
              
            //step four: run the path!
            test_path.cmd(),
            new AlignTowerPose(m_robotContainer.drivetrain)
        )
    );

    //m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees());
    return routine;
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


  /*
   * Limelight IMU Modes:
   * 0: No internal IMU processing. MT2 uses interpolated yaw from robot's gyro sent via SetRobotOrientation().
   * 1: Internal IMU offset is calibrated to match external yaw each frame (seeding). MT2 still uses external yaw for botpose.
   * 2: Uses internal IMU's fused yaw only. No external input required.
   * 3: Complementary filter fuses internal IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects internal IMU drift.
   * 4: Complementary filter fuses internal IMU with external yaw from SetRobotOrientation(). This is the recommended mode, as the internal IMU's 
   * 1khz update rate is utilized for frame-by-frame motion while the robot's IMU corrects for any drift over time.
   */

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    //m_robotContainer.drivetrain.getPigeon2().setYaw(LimelightHelpers.getBotPose3d_wpiBlue("limelight-front").getRotation().getAngle());
  }

  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 0);
    LimelightHelpers.SetThrottle("limelight-front", 200);
    
    AutoRoutine routine = autoFactory.newRoutine("TowerMcQueen");
    // Load the routine's trajectories
    // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("test");
    //AutoTrajectory test_path = routine.trajectory("TowerMcQueen");

    AutoTrajectory test_path = routine.trajectory("TowerMcQueen");

    new InstantCommand(() -> m_robotContainer.drivetrain.getPigeon2().setYaw(test_path.getInitialPose().get().getRotation().getDegrees()));
    //step two: reset odometry to starting pose
    test_path.resetOdometry();
    //step three: set LL heading to gyro (aka starting) heading
    new InstantCommand(() -> LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro
    LimelightHelpers.SetThrottle("limelight-front", 50);
  }

  @Override
  public void autonomousPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); // robot gyro
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetThrottle("limelight-front", 0);
    m_robotContainer.vision.setFirstVisionPose();

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

  @Override
  public void teleopPeriodic() {
    LimelightHelpers.SetIMUMode("limelight-front", 0); //robot gyro
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