// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Function;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.Xbox;
import frc.robot.Constants.BranchSide;
import frc.robot.Constants.ScoringLevel;
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

    // Replace with CommandXboxController, CommandPS4Controllerk or CommandJoystick if needed
    private CommandPS4Controller operatorController;
    private CommandGenericHID buttonBoxL;
    private CommandGenericHID buttonBoxR;
    private CommandXboxController joystick;

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
    public ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public RobotContainer() {
        NamedCommands.registerCommand("Remove Algae L2", algaeRemovalSequence(ScoringLevel.L2));
        NamedCommands.registerCommand("Remove Algae L3", algaeRemovalSequence(ScoringLevel.L3));
        NamedCommands.registerCommand("Intake Coral", autoIntakeCoralSequence());
        NamedCommands.registerCommand("Align and Score Coral Right L2", autoScoringSequence(BranchSide.RIGHT, ScoringLevel.L2));
        NamedCommands.registerCommand("Align and Score Coral Right L3", autoScoringSequence(BranchSide.RIGHT, ScoringLevel.L3));
        NamedCommands.registerCommand("Align and Score Coral Right L4", autoScoringSequence(BranchSide.RIGHT, ScoringLevel.L4));
        NamedCommands.registerCommand("Align and Score Coral Left L2", autoScoringSequence(BranchSide.LEFT, ScoringLevel.L2));
        NamedCommands.registerCommand("Align and Score Coral Left L4", autoScoringSequence(BranchSide.LEFT, ScoringLevel.L4));
        NamedCommands.registerCommand("Score Processor", autoScoreProcessorSequence()); //maybe change to 2.5
        NamedCommands.registerCommand("Center and Z Align", autoCenterAndZAlignSequence());

        // We defer joystick creation to initialization time so we can figure out if we have a button box or an operator controller
        // or both.
        joystick = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);  // We better have this one!
        operatorController = new CommandPS4Controller(Xbox.OPERATOR_CONTROLLER_PORT);
        if (DriverStation.isJoystickConnected(Xbox.BUTTON_BOX_LEFT) && DriverStation.isJoystickConnected(Xbox.BUTTON_BOX_RIGHT)) {
            buttonBoxL = new CommandGenericHID(Xbox.BUTTON_BOX_LEFT);
            buttonBoxR = new CommandGenericHID(Xbox.BUTTON_BOX_RIGHT);
        }

        // Make sure we've got something to operate the robot!

    
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
        getOperatorTrigger(CommandPS4Controller::R1, buttonBoxR, 11).whileTrue(scoreProcessorSequence());

        //CORAL INTAKE
        getOperatorTrigger(CommandPS4Controller::R2, buttonBoxL, 11).whileTrue(coralIntakeSequence());
        
        //ALGAE L2 REMOVAL
        getOperatorTrigger(CommandPS4Controller::povDown, buttonBoxL, 2).whileTrue(algaeRemovalSequence(ScoringLevel.L2));

        //ALGAE L3 REMOVAL
        getOperatorTrigger(CommandPS4Controller::povUp, buttonBoxL, 3).whileTrue(algaeRemovalSequence(ScoringLevel.L3));

        //ALGAE GROUND INTAKE
        getOperatorTrigger(CommandPS4Controller::L2, buttonBoxL, 4).whileTrue(algaeGroundIntakeSequence());

        //RIGHT BRANCH L2
        getOperatorTrigger(CommandPS4Controller::cross, buttonBoxR, 7).whileTrue(coralScoringSequence(BranchSide.RIGHT, ScoringLevel.L2));

        //RIGHT BRANCH L3
        getOperatorTrigger(CommandPS4Controller::square, buttonBoxR, 8).whileTrue(coralScoringSequence(BranchSide.RIGHT, ScoringLevel.L3));

        //RIGHT BRANCH L4
        getOperatorTrigger(CommandPS4Controller::triangle, buttonBoxR, 9).whileTrue(coralScoringSequence(BranchSide.RIGHT, ScoringLevel.L4)); 

        //LEFT BRANCH L2
        // No button box binding?!
        operatorController.cross().and(operatorController.L1()).whileTrue(coralScoringSequence(BranchSide.LEFT, ScoringLevel.L2));

        joystick.b().whileTrue(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism));

        //LEFT BRANCH L3
        getOperatorTrigger(CommandPS4Controller::square, CommandPS4Controller::L1, buttonBoxR, 3).whileTrue(coralScoringSequence(BranchSide.LEFT, ScoringLevel.L3));

        //LEFT BRANCH L4
        getOperatorTrigger(CommandPS4Controller::triangle, CommandPS4Controller::L1, buttonBoxR, 4).whileTrue(coralScoringSequence(BranchSide.LEFT, ScoringLevel.L4)); 
        

        //BARGE SCORING 
        getOperatorTrigger(CommandPS4Controller::povLeft, buttonBoxL, 8).whileTrue(bargeScoringSequence());

        joystick.b().whileTrue(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism));
        
        //REAL BARGE SCORING 
        getOperatorTrigger(CommandPS4Controller::circle, buttonBoxR, 1).whileTrue(realBargeScoringSequence());

        //BARGE SCORING
        //operatorController.circle().whileTrue(new BargeScoring(algaeMechanism, elevator, intake));
    }

    // Factory methods that return command sequences to be used in button bindings

    private Command realBargeScoringSequence() {
        return new SequentialCommandGroup(
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L1).withTimeout(1),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.SLOW).withTimeout(1)

            );
    }

    // If the above is "real" barge scoring, is this fake scoring?!
    private Command bargeScoringSequence() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT, algaeMechanism),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.BARGE_SCORING_ANGLE).withTimeout(2),
                new RunAlgaeIntake(intake, Constants.Algae.ALGAE_BARGE_OUTTAKE).withTimeout(5)
            )
        );
    }

    /**
     * Factory method to construct a coral scoring sequence given the side and level of the reef
     * to be scored on. Uses conditional commands to determine whether to run vision alignment as
     * part of the sequence.
     * @param side branch (left/right) to be scored on
     * @param level level of the reef to be scored on
     * @return command sequence that can be bound to button(s) to perform the scoring action
     * @implNote This sequence factory is *very* similar to the auto scoring factory. Auto uses different
     * fiducial offsets, but that can be handled in the helper method. This method runs the algae and
     * coral pivot moves in parallel, but auto does them serially; I suspect that auto could do
     * what this is doing and behave a litte faster. And this method uses timeouts that are a bit
     * more aggressive; again, auto could probably adopt those. Finally, auto doesn't run the last
     * part of the sequence in one particular case, but that's probably a bug; I suspect the 
     * difference in not intentional.
     */
    private Command coralScoringSequence(BranchSide side, ScoringLevel level) {
        double coralScoringAngle = getCoralScoringAngle(side, level);
        double algaeCoralScoringAngle = getAlgaeCoralScoringAngle(side, level);
        double elevatorHeight = getCoralScoringElevatorHeight(side, level);
        double fiducialOffset = getCoralScoringFiducialOffset(side, level, false);

        // WARNING! We can allow the driver to keep driving the robot through most of this command sequence by using a 
        // proxy command for the align portion of the command. It's critical that the subsystem(s) being used by any 
        // proxy commands are *only* used by other proxy commands in this sequence, or else the whole sequence will
        // cancel itself as soon as the proxied command starts running.
        Command conditionalAlignCommand = new ConditionalCommand(new AlignBranch(drivetrain, vision), new PrintCommand("Vision disabled; skipping alignment"), () -> !vision.isVisionDisabled());
        if (Constants.RobotBehavior.allowDriveInputWhileScoring) {
            conditionalAlignCommand = conditionalAlignCommand.asProxy();
        }

        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, fiducialOffset, 0.0)),
                new ConditionalCommand(new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism), new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism), () -> !vision.isVisionDisabled()),
                conditionalAlignCommand
            ),
            new MoveElevator(elevator, elevatorHeight, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, algaeCoralScoringAngle).withTimeout(1), 
                new MoveCoralPivot(coralPivot, coralScoringAngle).withTimeout(0.5)
            ),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2));
    }

    private Command algaeGroundIntakeSequence() {
        return new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.GROUND_INTAKE_HEIGHT),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        );
    }

    private Command algaeRemovalSequence(ScoringLevel level) {
        assert(level == ScoringLevel.L2 || level == ScoringLevel.L3);
        return new SequentialCommandGroup(
            new MoveElevator(elevator, level == ScoringLevel.L3 ? Constants.Elevator.ALGAE_L3_HEIGHT : Constants.Elevator.ALGAE_L2_HEIGHT, algaeMechanism),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        );
    }

    private Command coralIntakeSequence() {
        return new SequentialCommandGroup(
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
        );
    }

    private Command scoreProcessorSequence() {
        return new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.PROCESSOR_HEIGHT, algaeMechanism),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT), 
                new RunAlgaeIntake(intake, Constants.Algae.ALGAE_DEFAULT_SPEED)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_OUTTAKE_SPEED).withTimeout(2)
        );
    } 

    // Auto command factories

    private Command autoCenterAndZAlignSequence() {
        return new SequentialCommandGroup(
            new SequentialCommandGroup( // center align
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.0, Constants.Vision.LIMELIGHT_ALIGN_CENTER_OFFSET)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                new AlignBranch(drivetrain, vision)
            ),
            new SequentialCommandGroup(
                new ZAlign(drivetrain, vision).withTimeout(2)));
    }

    private Command autoScoreProcessorSequence() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT),
                new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2
        );
    }

    /**
     * Construct an auto scoring sequence for scoring on the given branch side and level.
     * @param side side of the reef to be scored on
     * @param level level to be scored on (must be L4/L3/L2)
     * @return command sequence to perform the scoring action
     */
    private Command autoScoringSequence(BranchSide side, ScoringLevel level) {
        double fiducialOffset = getCoralScoringFiducialOffset(side, level, true);
        double algaeAngle = getAlgaeCoralScoringAngle(side, level);
        double coralAngle = getCoralScoringAngle(side, level);
        double elevatorHeight = getCoralScoringElevatorHeight(side, level);

        // Not sure if this is intentional, but the right L2 scoring sequence omits the last part of the sequence used by 
        // all of the others. (Also, there is no left L3 sequence defined, but this method could now create one.)
        // I'm replicating this right L2 behavior, so you can see how one could do that, but it's probably a bug since the 
        // right L2 sequence doesn't do this.

        var commandSequence =  new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setFiducial3DOffset("limelight", 0.0, fiducialOffset, 0.0)),
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT, algaeMechanism),
                // new ZAlign(drivetrain, vision).withTimeout(3),
                new AlignBranch(drivetrain, vision)
            ),
            new MoveElevator(elevator, elevatorHeight, algaeMechanism),
            new MoveAlgaePivot(algaeMechanism, algaeAngle).withTimeout(1), 
            new MoveCoralPivot(coralPivot, coralAngle).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)
        );

        // Trailing sequence used by all command sequences except for right L2 for some reason...
        var additionalCommands = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunCoralOuttake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.SAFE, algaeMechanism)
        );

        // Add the trailing sequence to all command sequences except right/L2.
        if (!(side == BranchSide.RIGHT && level == ScoringLevel.L2)) {
            commandSequence.addCommands(additionalCommands);
        }

        return commandSequence;
    }

    private Command autoIntakeCoralSequence() {
        return new SequentialCommandGroup(
            //new ParallelRaceGroup(
                //new RunFunnel(funnel, Constants.Funnel.FUNNEL_SPEED_FAST),
                //new MoveElevator(elevator, Constants.Elevator.CORAL_STATION),
                new RunCoralIntake(coralIntake, Constants.CoralIntake.VELOCITY),//)
                new WaitCommand(0.1),
                new StageCoral(coralIntake, Constants.CoralIntake.INTAKE_ENCODER_STOP_VAL)
        );
    }
    
    // Helper methods

    /**
     * Returns the operator trigger to use as a binding for a command. It is given the Trigger
     * supplier for the operator controller, as well as the button box HID object, and the 
     * button number to use. Either the operator controller or the button box can be null,
     * but not both. If both are present, an or is used so that whatever is bound to the
     * returned triger will be executed if either trigger is triggered.
     * @param triggerSupplier the method reference to apply to a command controller, 
     * e.g., CommandPS4Controller:R1
     * @param buttonBoxHID the button box CommandGenericHID object (may be null)
     * @param button the button index to use for the binding on the button box HID
     * @return the (possibly combined) trigger to use for binding an action
     */
    private Trigger getOperatorTrigger(Function<CommandPS4Controller, Trigger> triggerSupplier, CommandGenericHID buttonBoxHID, int button) {
        Trigger operatorTrigger = null, buttonBoxTrigger = null;
        if (operatorController != null) {
            operatorTrigger = triggerSupplier.apply(operatorController);
        }
        if (buttonBoxHID != null) {
            buttonBoxTrigger = buttonBoxHID.button(button);
        }

        if (operatorTrigger != null && buttonBoxTrigger != null) {
            return operatorTrigger.or(buttonBoxTrigger);
        } else if (operatorTrigger != null) {
            return operatorTrigger;
        } else {
            return buttonBoxTrigger;
        }
    }

    /**
     * Returns the operator trigger to use as a binding for a command. It is given two Trigger
     * suppliers for the operator controller, both of which must be true, as well as the 
     * button box HID object, and the button number to use. Either the operator controller or 
     * the button box can be null, but not both. If both are present, an or is used so that 
     * whatever is bound to the returned triger will be executed if either trigger is triggered.
     * @param triggerSupplier1 the method reference to apply to a command controller, 
     * e.g., CommandPS4Controller:R1
     * @param triggerSupplier2 the method reference to apply to a command controller, 
     * e.g., CommandPS4Controller:L1
     * @param buttonBoxHID the button box CommandGenericHID object (may be null)
     * @param button the button index to use for the binding on the button box HID
     * @return the (possibly combined) trigger to use for binding an action
     */
    private Trigger getOperatorTrigger(Function<CommandPS4Controller, Trigger> triggerSupplier1, Function<CommandPS4Controller, Trigger> triggerSupplier2, CommandGenericHID buttonBoxHID, int button) {
        Trigger operatorTrigger = null, buttonBoxTrigger = null;
        if (operatorController != null) {
            operatorTrigger = triggerSupplier1.apply(operatorController).and(triggerSupplier2.apply(operatorController));
        }
        if (buttonBoxHID != null) {
            buttonBoxTrigger = buttonBoxHID.button(button);
        }

        if (operatorTrigger != null && buttonBoxTrigger != null) {
            return operatorTrigger.or(buttonBoxTrigger);
        } else if (operatorTrigger != null) {
            return operatorTrigger;
        } else {
            return buttonBoxTrigger;
        }
    }


    // We should convert some of these into a lookup tables

    /**
     * Compute the coral pivot angle we should use when attempting to socre on the 
     * given side and level of the reef.
     * @param side the branch side we intend to score on
     * @param level the branch level we intend to score on
     * @return the coral pivot angle to be used
     */
    private double getCoralScoringAngle(BranchSide side, ScoringLevel level) {
        double angle = Constants.CoralPivot.SAFE;
        switch (level) {
            case L4:
                angle = Constants.CoralPivot.L4;
                break;
            case L3:
            case L2:
                angle = Constants.CoralPivot.L2L3;
                break;
            default:
                System.out.println("Unhandled coral scoring angle!");
                break;
        }

        return angle;
    }

    /**
     * Compute the algae pivot angle we should use when attempting to socre on the 
     * given side and level of the reef.
     * @param side the branch side we intend to score on (currently ignored)
     * @param level the branch level we intend to score on
     * @return the algae pivot angle to be used
     */
    private double getAlgaeCoralScoringAngle(BranchSide side, ScoringLevel level) {
        double angle = Constants.Algae.SAFE;
        switch (level) {
            case L4:
                angle = Constants.Algae.L4_CORAL_SCORING_ANGLE;
                break;
            case L3:
            case L2:
                angle = Constants.Algae.L2L3_CORAL_SCORING_ANGLE;
                break;
            default:
                System.out.println("Unhandled scoring angle for algae!");
                break;
        }

        return angle;
    }

    /**
     * Compute the elevator height to score coral on a given side and level. This method
     * currently ignores the side, but the value is passed in case we need to make 
     * adjustments. We could even have this method take the reef face as an argument if
     * we found we needed to make per-face adjustments.
     * @param side the branch side we intend to score on (currently ignored)
     * @param level the branch level we intend to score on
     * @return the elevator height to accomplish scoring
     */
    private double getCoralScoringElevatorHeight(BranchSide side, ScoringLevel level) {
        double height = Constants.Elevator.SAFE;
        switch (level) {
            case L4:
                height = Constants.Elevator.CORAL_L4_HEIGHT;
                break;
            case L3:
                height = Constants.Elevator.CORAL_L3_HEIGHT;
                break;
            case L2:
                height = Constants.Elevator.CORAL_L2_HEIGHT;
                break;
            case L1:
            default:
                System.out.println("Unhandled scoring height!");
                break;
        }

        return height;
    }

    /**
     * Compute the fiducial offset we should use when attempting to socre on the 
     * given side and level of the reef.
     * @param side the branch side we intend to score on
     * @param level the branch level we intend to score on
     * @return the fiducial offset to accomplish scoring alignment
     */
    private double getCoralScoringFiducialOffset(BranchSide side, ScoringLevel level, boolean isAuto) {
        // There are few enough cases, this would be simpler with an if tree, but this makes it easier for us to add other special
        // cases. 
        double offset = 0.0;
        if (isAuto) {
            offset = side == BranchSide.LEFT ? Constants.Vision.LIMELIGHT_AUTO_LEFT_OFFSET : Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET;
        } else {
            switch (level) {
                case L4:
                    offset = side == BranchSide.RIGHT ? Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET : Constants.Vision.LIMELIGHT_L4_LEFT_OFFSET;
                    break;
                case L3:
                case L2:
                    offset = side == BranchSide.RIGHT ? Constants.Vision.LIMELIGHT_ALIGN_RIGHT_OFFSET : Constants.Vision.LIMELIGHT_ALIGN_LEFT_OFFSET;
                    break;
                case L1:
                default:
                    System.out.println("Unhandled scoring fiducial offset!");
                    break;
            }
        }

        return offset;
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