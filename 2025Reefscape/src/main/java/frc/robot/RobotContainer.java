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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final CommandPS4Controller operatorController = new CommandPS4Controller(Xbox.OPERATOR_CONTROLLER_PORT);

    //private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(2);

    public final Intake intake = new Intake();
    public final Climb climb = new Climb();
    public final Pivot pivot = new Pivot();

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");

    public RobotContainer() {
        NamedCommands.registerCommand("Coral Ground Intake", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_GROUND_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        NamedCommands.registerCommand("Coral L1", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        NamedCommands.registerCommand("Algae Intake", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        NamedCommands.registerCommand("Algae Processor", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_PROCESSOR),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        NamedCommands.registerCommand("Algae Descore", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        NamedCommands.registerCommand("Climb", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.PRE_CLIMB),
            new RunClimb(climb, Constants.Climb.VELOCITY),
            new MovePivot(pivot, Constants.Pivot.CLIMB)
        ));

        configureBindings();
        intake.configDashboard(matchTab);
        climb.configDashboard(matchTab);
        pivot.configDashboard(matchTab);

    }
    
    private void configureBindings() {
        //DEFAULT COMMANDS
        pivot.setDefaultCommand(new MovePivot(pivot, Constants.Pivot.PRE_CLIMB));
        intake.setDefaultCommand(new RunIntake(intake, 0.0));
        
        //GROUND INTAKE
        operatorController.R2().or(operatorXbox.rightTrigger()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_GROUND_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        //CORAL L1 
        operatorController.cross().or(operatorXbox.a()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //ALGAE INTAKE
        operatorController.L2().or(operatorXbox.leftTrigger()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_INTAKE),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //ALGAE PROCESSOR
        operatorController.R1().or(operatorXbox.rightBumper()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_PROCESSOR),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        //ALGAE DESCORE
        operatorController.povDown().or(operatorXbox.povDown()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //CLIMB
        operatorController.square().or(operatorXbox.x()).whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.PRE_CLIMB),
            new RunClimb(climb, Constants.Climb.VELOCITY),
            new MovePivot(pivot, Constants.Pivot.CLIMB)
        ));
    } 
}