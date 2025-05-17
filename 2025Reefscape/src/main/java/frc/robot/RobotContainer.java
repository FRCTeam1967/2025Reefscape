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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
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
import frc.robot.generated.TunerConstants;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public boolean visionEnabled = true;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS4Controller operatorController = new CommandPS4Controller(Xbox.OPERATOR_CONTROLLER_PORT);

    private final CommandGenericHID buttonBoxL = new CommandGenericHID(2);
    private final CommandGenericHID buttonBoxR = new CommandGenericHID(3);

    //private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Elevator elevator = new Elevator();
    public final AlgaePivot algaeMechanism = new AlgaePivot();
    public final AlgaeIntake intake = new AlgaeIntake();
    public final LEDSubsystem led = new LEDSubsystem();

    public final CoralPivot coralPivot = new CoralPivot();
    public final static CoralIntake coralIntake = new CoralIntake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Vision vision = new Vision(); 
    public static final Vision odometryVision = new Vision();
    // public static final VisionUpdate update = new VisionUpdate(drivetrain);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /* Path follower */
    public static SendableChooser<Command> autoChooserLOL;

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public RobotContainer() {
        NamedCommands.registerCommand("Remove Algae L2", new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L2_HEIGHT, algaeMechanism),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)));
        
            NamedCommands.registerCommand("Remove Algae L3", new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L3_HEIGHT, algaeMechanism),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        NamedCommands.registerCommand("Intake Coral", new SequentialCommandGroup(
            //new ParallelRaceGroup(
                //new RunFunnel(funnel, Constants.Funnel.FUNNEL_SPEED_FAST),
                //new MoveElevator(elevator, Constants.Elevator.CORAL_STATION),
                new RunCoralIntake(coralIntake, Constants.CoralIntake.VELOCITY),//)
                new WaitCommand(0.1),
            new StageCoral(coralIntake, Constants.CoralIntake.INTAKE_ENCODER_STOP_VAL)
        ));

        NamedCommands.registerCommand("Intake Coral", new SequentialCommandGroup(
            //new ParallelRaceGroup(
                //new RunFunnel(funnel, Constants.Funnel.FUNNEL_SPEED_FAST),
                //new MoveElevator(elevator, Constants.Elevator.CORAL_STATION),
                new RunCoralIntake(coralIntake, Constants.CoralIntake.VELOCITY),//)
                new WaitCommand(0.1),
            new StageCoral(coralIntake, Constants.CoralIntake.INTAKE_ENCODER_STOP_VAL)
        ));
        
        NamedCommands.registerCommand("Align and Score Coral Right L2", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                new AlignBranch(drivetrain, vision)),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));
        
        NamedCommands.registerCommand("Align and Score Coral Right L3",new SequentialCommandGroup(
            new SequentialCommandGroup( // horizontal align
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                new AlignBranch(drivetrain, vision)),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2),
            new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism)));

        NamedCommands.registerCommand("Align and Score Coral Right L4", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                // new ZAlign(drivetrain, vision).withTimeout(3),
                new AlignBranch(drivetrain, vision)),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE))
                .withTimeout(2),
                new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism)));
            
        
        NamedCommands.registerCommand("Align and Score Coral Left L2", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_LEFT_OFFSET, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                new AlignBranch(drivetrain, vision)),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE))
                .withTimeout(2),
                new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism)));
        
        NamedCommands.registerCommand("Align and Score Coral Left L4", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_AUTO_LEFT_OFFSET, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                // new ZAlign(drivetrain, vision).withTimeout(3),
                new AlignBranch(drivetrain, vision)),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE))
                .withTimeout(3),
                new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism)));

        NamedCommands.registerCommand("Score Processor", new SequentialCommandGroup(
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT),
            new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2)); //maybe change to 2.5

        NamedCommands.registerCommand("Center and Z Align", new SequentialCommandGroup(
            new SequentialCommandGroup( // center align
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.0, Constants.Vision.LIMELIGHT_ALIGN_CENTER_OFFSET)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                new AlignBranch(drivetrain, vision)),
            new SequentialCommandGroup(
                new ZAlign(drivetrain, vision).withTimeout(3))));
        
    
        vision.configDashboard(matchTab);
        vision.configLLTab(limelightTab);
        
        autoChooserLOL = AutoBuilder.buildAutoChooser();
        fieldTab.add("Field", CommandSwerveDrivetrain.m_field).withWidget(BuiltInWidgets.kField).withSize(8, 4);

        //limelightTab.addString("limelight stream", () -> "http://10.19.67.12:5801/")


        configureBindings();
        drivetrain.configDashboard(matchTab);
        elevator.configDashboard(matchTab);
        algaeMechanism.configDashboard(matchTab);
        coralPivot.configDashboard(matchTab);
        coralIntake.configDashboard(matchTab);
        
        //algaeMechanism.setReltoAbs();
    }
    
    private void configureBindings() {
        // Note that X is defdrivetrain.applyRequestined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            (() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );*/
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                    double currMaxSpeed = MaxSpeed;
                    double currAngularRate = MaxAngularRate;

                     if (elevator.getHeight() >= 2){
                         currMaxSpeed = MaxSpeed/5;
                        
                         currAngularRate = MaxAngularRate/5;
                     }else{
                         currMaxSpeed = MaxSpeed;
                         currAngularRate = MaxAngularRate;
                    }

                    return drive.withVelocityX(-joystick.getLeftY() * currMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * currMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * currAngularRate); // Drive counterclockwise with negative X (left)
                })
        );

        //spin to get coral off
        joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ))); // Drive counterclockwise with negative X (left)}))))
                    
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
            //point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.x().onTrue(new SequentialCommandGroup( // z align
        new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.0, Constants.Vision.LIMELIGHT_ALIGN_Z_OFFSET)),
        new ZAlign(drivetrain, vision)));

        //DEFAULT COMMANDS
        algaeMechanism.setDefaultCommand(new MoveAlgaePivot(algaeMechanism, Constants.Algae.SAFE));
        elevator.setDefaultCommand(new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism));
        coralPivot.setDefaultCommand(new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE));
        intake.setDefaultCommand(new RunAlgaeIntake(intake, -0.03));
        led.setDefaultCommand(new BlackLED(led));

        //RESET GYRO
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0.2 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-0.2 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.povRight().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-0.2 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        joystick.povLeft().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0.2 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        //DRIVER CONTROLLER VISION ALIGNMENT
        joystick.rightBumper().onTrue(new SequentialCommandGroup(
            //new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
            new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
            new AlignBranch(drivetrain, vision)));

        joystick.leftBumper().onTrue(new SequentialCommandGroup(
            //new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_LEFT_OFFSET, 0.0)),
            new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
            new AlignBranch(drivetrain, vision)));

        //DISABLE VISION
        joystick.x().onTrue(new InstantCommand(() -> vision.disableVision(), vision));
        
        //SCORE PROCESSOR
        buttonBoxR.button(11).or(operatorController.R1()).whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.PROCESSOR_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT), 
                new RunAlgaeIntake(intake, Constants.Algae.ALGAE_DEFAULT_SPEED)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_OUTTAKE_SPEED).withTimeout(2)
        ));

        //CORAL INTAKE
        buttonBoxL.button(11).or(operatorController.R2()).whileTrue(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.CORAL_INTAKE),
                new RunCoralIntake(coralIntake, Constants.CoralIntake.VELOCITY)
            ),
            new ParallelCommandGroup(
                new RunCoralIntakeBack(coralIntake, Constants.CoralIntake.REVERSE_VELOCITY),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.CORAL_REVERSE_INTAKE)
            ),
            new RumbleController(joystick, operatorController).withTimeout(2)
            //new WaitCommand(0.1),
            //new StageCoral(coralIntake, Constants.CoralIntake.INTAKE_ENCODER_STOP_VAL)
            //new RunCoralSecondIntake(coralIntake, Constants.CoralIntake.VELOCITY)
        ));
        
        //ALGAE L2 REMOVAL
        buttonBoxL.button(2).or(operatorController.povDown()).whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L2_HEIGHT, algaeMechanism),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //ALGAE L3 REMOVAL
        buttonBoxL.button(3).or(operatorController.povUp()).whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L3_HEIGHT, algaeMechanism),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //ALGAE GROUND INTAKE
        buttonBoxL.button(4).or(operatorController.L2()).whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.GROUND_INTAKE_HEIGHT),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //RIGHT BRANCH L2
        buttonBoxR.button(7).or(operatorController.cross()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //RIGHT BRANCH L3
        buttonBoxR.button(8).or(operatorController.square()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //RIGHT BRANCH L4
        buttonBoxR.button(9).or(operatorController.triangle()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),//.asProxy()),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2))); //maybe change to 2.5

        //LEFT BRANCH L2
        operatorController.cross().and(operatorController.L1()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_LEFT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        joystick.b().whileTrue(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism));

        //LEFT BRANCH L3
        buttonBoxR.button(3).or(operatorController.square().and(operatorController.L1())).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_ALIGN_LEFT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //LEFT BRANCH L4
        buttonBoxR.button(4).or(operatorController.triangle().and(operatorController.L1())).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, Constants.Vision.LIMELIGHT_L4_LEFT_OFFSET, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism), () -> !vision.isVisionDisabled()),
                new ConditionalCommand(new AlignBranch(drivetrain, vision), new WaitCommand(0.0), () -> !vision.isVisionDisabled())),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(0.5)),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2))); //maybe change to 2.5
        

        //BARGE SCORING 
        buttonBoxL.button(8).or(operatorController.povLeft()).whileTrue(
        new SequentialCommandGroup(
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.BARGE_SCORING_ANGLE).withTimeout(2),
            new ParallelCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.BARGE_SCORING_ALMOST_HEIGHT, algaeMechanism),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2),
            new ParallelCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.BARGE_SCORING_HEIGHT, algaeMechanism),
                new RunAlgaeIntake(intake, Constants.Algae.ALGAE_BARGE_OUTTAKE)
            ).withTimeout(5)
        ));

        joystick.b().whileTrue(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism));
        
        //L1 SCORING 
        buttonBoxR.button(1).or(operatorController.circle()).whileTrue(new SequentialCommandGroup(
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L1).withTimeout(1),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.SLOW).withTimeout(1)));

        //BARGE SCORING
        //operatorController.circle().whileTrue(new BargeScoring(algaeMechanism, elevator, intake));
    } 
    

    public Command getAutonomousCommand() {
        return autoChooserLOL.getSelected();
    }
    
    /******************************************/

    // //limelight methods for alignment
    // //for X alignment (how rotational it should align)
    // private double limelight_aim_proportional() {
    //     double kP = 0.1; //test -> fix large errors
    //     double kI = 0.2; //test -> reduce steady-state error (+ oscillation)
    //     double kD = 0.2; //test -> slow down when reaching target (stability)
        
    //     //TX -> x-axis offset in degrees, multiply by angular speed to be radians/second
    //     double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") * kP) * CommandSwerveDrivetrain.kMaxAngularSpeed;
        
    //     targetingAngularVelocity *= 1.0;
    //     return targetingAngularVelocity;
    // }

    // //for Y alignment (how forward/backward it should go)
    // private double limelight_range_proportional() {    
    //     double kP = 0.02; //test

    //     //TY -> y-axis offset in degrees, multiply by angular speed to be raidans/second
    //     double targetingForwardSpeed = (LimelightHelpers.getTY("limelight") * kP) * CommandSwerveDrivetrain.kMaxSpeed;

    //     targetingForwardSpeed *= -1.0;
    //     return targetingForwardSpeed;
    // }

    // //drive for robot container
    // public void drive(boolean fieldRelative) {
    //     var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftY(), 0.02)) * CommandSwerveDrivetrain.kMaxSpeed;
    //     var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftX(), 0.02)) * CommandSwerveDrivetrain.kMaxSpeed;
    //     var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getRightX(), 0.02)) * CommandSwerveDrivetrain.kMaxAngularSpeed;

    //     // while the left-bumper is pressed, overwrite some of the driving values with the output of limelight override method
    //     if (joystick.leftTrigger().getAsBoolean()){
    //         final var rot_limelight = limelight_aim_proportional();
    //         rot = rot_limelight;

    //         final var forward_limelight = limelight_range_proportional();
    //         xSpeed = forward_limelight;

    //         //turn off field relative
    //         fieldRelative = false;
    //     }

    // drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, 1);
    // }
    
}