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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Xbox;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandPS4Controller operatorController = new CommandPS4Controller(Xbox.OPERATOR_CONTROLLER_PORT);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Intake intake = new Intake();
    public final Climb climb = new Climb();
    public final Pivot pivot = new Pivot();
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");

    public static SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Coral Ground Intake", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_GROUND_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        NamedCommands.registerCommand("Coral L1", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();/*
        intake.configDashboard(matchTab);
        climb.configDashboard(matchTab);
        pivot.configDashboard(matchTab);*/

    }
    
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                })
        );
        
        //DEFAULT COMMANDS
        pivot.setDefaultCommand(new MovePivot(pivot, Constants.Pivot.PRE_CLIMB));
        intake.setDefaultCommand(new RunIntake(intake, 0.0));

        joystick.start().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );
        
        //GROUND INTAKE
        operatorController.R2().whileTrue(new SequentialCommandGroup(
            //new MovePivot(pivot, Constants.Pivot.CORAL_GROUND_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        //CORAL L1 
        operatorController.cross().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //ALGAE INTAKE
        operatorController.L2().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_INTAKE),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //ALGAE PROCESSOR
        operatorController.R1().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_PROCESSOR),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        //ALGAE DESCORE
        operatorController.povDown().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //CLIMB
        operatorController.square().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.PRE_CLIMB),
            new RunClimb(climb, Constants.Climb.VELOCITY).withTimeout(3),
            new MovePivot(pivot, Constants.Pivot.CLIMB)
        ));

    } 
}