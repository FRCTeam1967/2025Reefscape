// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Xbox;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Claw254 claw = new Claw254();
  private final Intake254 intake = new Intake254();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);

  public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {            
    // Configure the trigger bindings
    configureBindings();

    claw.configDashboard(matchTab);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    claw.setDefaultCommand(new MoveClaw(claw, Constants.Intake.CLAW_OPEN));
    // top roller does not spin currently for the cargo because it is not fully intaking & spinning causes the ball to launch out + bend the claw
    // intake cargo ball
    operatorXbox.rightTrigger().whileTrue(new SequentialCommandGroup(
      new MoveClaw(claw, Constants.Intake.CLAW_OPEN),
      new ParallelCommandGroup(new RunRollers(intake, Constants.Intake.SIDE_INTAKE_SPEED, Constants.Intake.TOP_STOP_SPINNING)), new MoveClaw(claw, Constants.Intake.CLAW_OPEN)));

    // outtake cargo ball
    operatorXbox.y().whileTrue(new SequentialCommandGroup(
      new MoveClaw(claw, Constants.Intake.CLAW_OPEN),
      new ParallelCommandGroup(new RunRollers(intake, Constants.Intake.SIDE_OUTTAKE_SPEED, Constants.Intake.TOP_OUTTAKE_SPEED)), new MoveClaw(claw, Constants.Intake.CLAW_OPEN)));
    
    // intake hatch panel
    operatorXbox.leftTrigger().whileTrue(new SequentialCommandGroup(
      new MoveClaw(claw, Constants.Intake.CLAW_CLOSE),
      new RunRollers(intake, Constants.Intake.SIDE_OUTTAKE_SPEED, Constants.Intake.SIDE_STOP_SPINNING), new MoveClaw(claw, Constants.Intake.CLAW_CLOSE)));

    // outtake hatch panel
    operatorXbox.x().whileTrue(new SequentialCommandGroup(
      new MoveClaw(claw, Constants.Intake.CLAW_CLOSE),
      new RunRollers(intake, Constants.Intake.SIDE_INTAKE_SPEED, Constants.Intake.TOP_STOP_SPINNING), new MoveClaw(claw, Constants.Intake.CLAW_CLOSE)));

    operatorXbox.a().onTrue(new MoveClaw(claw, Constants.Intake.CLAW_OPEN));
    operatorXbox.b().onTrue(new MoveClaw(claw, Constants.Intake.CLAW_CLOSE));
  }
}
