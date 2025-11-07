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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    //private final CommandPS4Controller operatorController = new CommandPS4Controller(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    public Trigger algaeStartTrigger;
    public Trigger algaeHoldTrigger;
    public Trigger algaeEndTrigger;

    public RobotContainer() {
        NamedCommands.registerCommand("Drive Forward",
            new DriveForward(drivetrain).withTimeout(1)
        );
        
        NamedCommands.registerCommand("Coral L1 Score", new SequentialCommandGroup(
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY).withTimeout(2)
        ));
        NamedCommands.registerCommand("Scoring Position", new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1).withTimeout(0.5)
        ));
        NamedCommands.registerCommand("Backward", new SequentialCommandGroup(
            new DriveBackward(drivetrain).withTimeout(1)
        ));
        autoChooser = AutoBuilder.buildAutoChooser();

        matchTab.add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);

        //double pivotPosition = pivot.getPosition()/Constants.Pivot.GEAR_RATIO*360;
        algaeStartTrigger = new Trigger(() -> (pivot.getPosition() > Constants.Pivot.ALGAE_INTAKE_THRESHOLD));
        algaeEndTrigger= new Trigger(() -> (pivot.getPosition() < Constants.Pivot.ALGAE_STEP_3));
        algaeHoldTrigger= new Trigger(() -> (pivot.getPosition() < Constants.Pivot.ALGAE_STEP_4));

        configureBindings();
        intake.configDashboard(matchTab);
        //climb.configDashboard(matchTab);
        pivot.configDashboard(matchTab);
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
        pivot.setDefaultCommand(new MovePivot(pivot, 0.0));
        intake.setDefaultCommand(new RunIntake(intake, Constants.Intake.IDLE_SPEED));

        joystick.start().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );
        
        //GROUND INTAKE
        operatorController.rightTrigger().whileTrue(new ParallelCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_GROUND_INTAKE),
            new RunIntake(intake, Constants.Intake.INTAKE_SPEED))
        );

        //CORAL L1 
        operatorController.x().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.CORAL_L1).withTimeout(1.23),
            new RunIntake(intake, Constants.Intake.EJECT_VELOCITY))
        );

        //ALGAE INTAKE
        /*operatorController.leftTrigger().whileTrue(new ParallelCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_INTAKE).withTimeout(1.5),
            new RunIntake(intake, Constants.Intake.ALGAE_INTAKE_SPEED))
        );*/

        //EJECT ALGAE
        operatorController.y().whileTrue(new SequentialCommandGroup(
            new RunIntake(intake, Constants.Intake.ALGAE_EJECT_SPEED))
        );

        // PROCESSOR SCORING SEQUENCE
        operatorController.leftTrigger().and(algaeStartTrigger.and(algaeEndTrigger)).whileTrue(
          new RunIntake(intake, Constants.Intake.ALGAE_INTAKE_SPEED)
        );

        operatorController.leftTrigger().and(algaeHoldTrigger).whileTrue(
          new RunIntake(intake, 0.0)
        );

        operatorController.leftTrigger().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_INTAKE).withTimeout(2.0),
            new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_2).withTimeout(0.5),
            new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_3).withTimeout(0.5),
            new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_4)//.withTimeout(0.5),
            //new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_5).withTimeout(0.5),
            //new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_6).withTimeout(0.5),
            //new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_7).withTimeout(0.5),
            //new MovePivot(pivot, Constants.Pivot.ALGAE_STEP_8)
        ));

        //ALGAE PROCESSOR
        operatorController.povLeft().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_PROCESSOR).withTimeout(2),
            new RunIntake(intake, Constants.Intake.ALGAE_EJECT_SPEED))
        );

        //ALGAE DESCORE
        operatorController.povDown().whileTrue(new SequentialCommandGroup(
            new MovePivot(pivot, Constants.Pivot.ALGAE_DESCORE).withTimeout(1.5),
            new RunIntake(intake, Constants.Intake.DESCORE_VELOCITY))
        );

        //PRE CLIMB
        operatorController.a().onTrue(
            new MovePivot(pivot, Constants.Pivot.PRE_CLIMB)
        );

        // CLIMB
        operatorController.b().whileTrue(new SequentialCommandGroup(
            new RunClimb(climb, Constants.Climb.VELOCITY)
        ));

        operatorController.povUp().onTrue(new ParallelCommandGroup(
            new RunClimb(climb, Constants.Climb.VELOCITY),
            new MovePivot(pivot, Constants.Pivot.CLIMB)
        ));

        //ZERO
        joystick.a().onTrue(new SequentialCommandGroup(
            new MovePivot(pivot, 0.0).withTimeout(1.5)
        ));

        joystick.b().onTrue(new SequentialCommandGroup(
            new RunClimb(climb, Constants.Climb.CLOSING_VELOCITY).withTimeout(1.5)
        ));
        
    } 

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}