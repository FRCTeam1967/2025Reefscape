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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Elevator elevator = new Elevator();
    public final AlgaePivot algaeMechanism = new AlgaePivot();
    public final AlgaeIntake intake = new AlgaeIntake();

    public final static CoralPivot coralPivot = new CoralPivot();
    public final static CoralIntake coralIntake = new CoralIntake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Vision vision = new Vision("limelight"); 
    public static final Vision odometryVision = new Vision("limelight-santos");

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final VisionUpdate visionUpdate = new VisionUpdate(drivetrain, "limelight-santos");

    /* Path follower */
    public static SendableChooser<Command> autoChooserLOL;

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
    public ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    public RobotContainer() {
        NamedCommands.registerCommand("Align and Score Coral Right L2", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignRightBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        NamedCommands.registerCommand("Score Coral L3", new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT),
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3)));
        NamedCommands.registerCommand("Run CoralIntake", new RunCoralIntake(coralIntake, Constants.CoralIntake.SLOW).withTimeout(1.0));

        NamedCommands.registerCommand("AlignAndScoreCoralLeftL4", new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
            new AlignLeftBranch(drivetrain, vision).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE))
                .withTimeout(2)));

        NamedCommands.registerCommand("shootAlgae", new SequentialCommandGroup(
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT),
            new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_OUTTAKE_SPEED)));

        vision.configDashboard(matchTab);
        odometryVision.odometryConfigDashboard(matchTab);
        
        autoChooserLOL = AutoBuilder.buildAutoChooser();
        fieldTab.add("Field", CommandSwerveDrivetrain.m_field).withWidget(BuiltInWidgets.kField).withSize(8, 4);
        
        HttpCamera httpCamera1 = new HttpCamera("limelight", "http://10.19.67.12:5801/"); //http://10.19.67.202:5801/
        CameraServer.addCamera(httpCamera1);

        HttpCamera httpCamera2 = new HttpCamera("limelight-santos", "http://10.19.67.11:5801/"); //http://10.19.67.11:5801/
        CameraServer.addCamera(httpCamera2);

        limelightTab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
        .withSize(3, 2);
        limelightTab.add(httpCamera2).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0)
        .withSize(3, 2);


        configureBindings();
        drivetrain.configDashboard(matchTab);
        elevator.configDashboard(matchTab);
        algaeMechanism.configDashboard(matchTab);
        coralPivot.configDashboard(matchTab);
        coralIntake.configDashboard(matchTab);
        
        //algaeMechanism.setReltoAbs();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
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

        //DEFAULT COMMANDS
        algaeMechanism.setDefaultCommand(new MoveAlgaePivot(algaeMechanism, Constants.Algae.SAFE));
        elevator.setDefaultCommand(new MoveElevator(elevator, Constants.Elevator.SAFE));
        coralPivot.setDefaultCommand(new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE));
        intake.setDefaultCommand(new RunAlgaeIntake(intake, -0.15));

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
            new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
            new AlignRightBranch(drivetrain, vision)).withTimeout(3));

        joystick.leftBumper().onTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
            new AlignLeftBranch(drivetrain, vision)).withTimeout(3));

        //DISABLE VISION
        joystick.y().onTrue(new InstantCommand(() -> vision.disableVision(), vision));
        
        //SCORE PROCESSOR
        operatorController.rightBumper().whileTrue(new SequentialCommandGroup(
            new ParallelRaceGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.PROCESSOR_HEIGHT), 
                new RunAlgaeIntake(intake, Constants.Algae.ALGAE_DEFAULT_SPEED))
            .withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_OUTTAKE_SPEED).withTimeout(2)
        ));

        //CORAL INTAKE
        operatorController.rightTrigger().whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.CORAL_STATION),
            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.CORAL_SCORING_ANGLE),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.CORAL_STATION_INTAKE_ANGLE)).withTimeout(1),
            new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(0.07),
            new ParallelCommandGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.EXTRA_CORAL_SCORING_ANGLE), new RunAlgaeIntake(intake, -0.1))));

        //ALGAE L2 REMOVAL
        operatorController.povDown().whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L2_HEIGHT),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //ALGAE L3 REMOVAL
        operatorController.povUp().whileTrue(new SequentialCommandGroup(
            new MoveElevator(elevator, Constants.Elevator.ALGAE_L3_HEIGHT),
            new ParallelRaceGroup(new MoveAlgaePivot(algaeMechanism, Constants.Algae.ALGAE_DOWN), new RunAlgaeIntake(intake, -0.7)).withTimeout(0.75),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //ALGAE GROUND INTAKE
        operatorController.leftTrigger().whileTrue(new SequentialCommandGroup(
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.GROUND_INTAKE_HEIGHT),
            new RunAlgaeIntake(intake, Constants.Algae.ALGAE_INTAKE_SPEED)
        ));

        //RIGHT BRANCH L2
        operatorController.a().whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignRightBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //RIGHT BRANCH L3
        operatorController.x().whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignRightBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //RIGHT BRANCH L4
        operatorController.y().whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignRightBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2))); //maybe change to 2.5

        //LEFT BRANCH L2
        operatorController.a().and(operatorController.leftBumper()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignLeftBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L2_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //LEFT BRANCH L3
        operatorController.x().and(operatorController.leftBumper()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignLeftBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L3_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L2L3_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L2L3).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1)));

        //LEFT BRANCH L4
        operatorController.y().and(operatorController.leftBumper()).whileTrue(new SequentialCommandGroup(
            new SequentialCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.VISION_HEIGHT),
                new AlignLeftBranch(drivetrain, vision)).withTimeout(3),
            new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT),
            new MoveAlgaePivot(algaeMechanism, Constants.Algae.L4_CORAL_SCORING_ANGLE).withTimeout(1), 
            new MoveCoralPivot(coralPivot, Constants.CoralPivot.L4).withTimeout(1),
            //new RunCoralIntake(coralIntake, Constants.CoralIntake.HIGH),
            new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH).withTimeout(1),
            new ParallelCommandGroup(
                new RunCoralFastIntake(coralIntake, Constants.CoralIntake.HIGH),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2))); //maybe change to 2.5
        

        operatorController.b().whileTrue(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new MoveElevator(elevator, Constants.Elevator.CORAL_L4_HEIGHT),
                new MoveCoralPivot(coralPivot, Constants.CoralPivot.SAFE)
            ).withTimeout(2),

            new ParallelCommandGroup(
                new MoveAlgaePivot(algaeMechanism, Constants.Algae.BARGE_SCORING_ANGLE).withTimeout(2),
                new RunAlgaeIntake(intake, Constants.Algae.BARGE_SCORING_SPEED).withTimeout(5)
            )
        )
        );
        

        //L1 CORAL SCORING   
        // operatorController.b().whileTrue(new SequentialCommandGroup(
        //     new RunCoralFastIntake(coralIntake, Constants.CoralIntake.SLOW).withTimeout(1),
        //     new MoveCoralPivot(coralPivot, Constants.CoralPivot.L1)));
    }


    public Command getAutonomousCommand() {
        return autoChooserLOL.getSelected();
    }
    
    /******************************************/

    //limelight methods for alignment
    //for X alignment (how rotational it should align)
    private double limelight_aim_proportional() {
        double kP = 0.1; //test -> fix large errors
        double kI = 0.2; //test -> reduce steady-state error (+ oscillation)
        double kD = 0.2; //test -> slow down when reaching target (stability)
        
        //TX -> x-axis offset in degrees, multiply by angular speed to be radians/second
        double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") * kP) * CommandSwerveDrivetrain.kMaxAngularSpeed;
        
        targetingAngularVelocity *= 1.0;
        return targetingAngularVelocity;
    }

    //for Y alignment (how forward/backward it should go)
    private double limelight_range_proportional() {    
        double kP = 0.02; //test

        //TY -> y-axis offset in degrees, multiply by angular speed to be raidans/second
        double targetingForwardSpeed = (LimelightHelpers.getTY("limelight") * kP) * CommandSwerveDrivetrain.kMaxSpeed;

        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    //drive for robot container
    public void drive(boolean fieldRelative) {
        var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftY(), 0.02)) * CommandSwerveDrivetrain.kMaxSpeed;
        var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftX(), 0.02)) * CommandSwerveDrivetrain.kMaxSpeed;
        var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getRightX(), 0.02)) * CommandSwerveDrivetrain.kMaxAngularSpeed;

        // while the left-bumper is pressed, overwrite some of the driving values with the output of limelight override method
        if (joystick.leftTrigger().getAsBoolean()){
            final var rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;

            final var forward_limelight = limelight_range_proportional();
            xSpeed = forward_limelight;

            //turn off field relative
            fieldRelative = false;
        }

        drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, 1);

    }
}
