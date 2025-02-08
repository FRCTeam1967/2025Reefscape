// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final static Elevator elevator = new Elevator();
    public final AlgaeMechanism algaeMechanism = new AlgaeMechanism();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Vision vision = new Vision("limelight"); 

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final VisionUpdate visionUpdate = new VisionUpdate(drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooserLOL;

    public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");

    public RobotContainer() {
        autoChooserLOL = AutoBuilder.buildAutoChooser();
        matchTab.add("Auto Chooser lol", autoChooserLOL).withWidget(BuiltInWidgets.kComboBoxChooser);
        fieldTab.add("Field", VisionUpdate.m_field)
            .withWidget(BuiltInWidgets.kField)
            .withSize(8, 4);

        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
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

        operatorController.y().onTrue(new MoveElevator(elevator, Constants.Elevator.UP));
        operatorController.x().onTrue(new MoveElevator(elevator, Constants.Elevator.MIDDLE));
        //operatorController.a().onTrue(new ElevatorSafe(elevator, Constants.Elevator.SAFE));

        operatorController.a().onTrue(new MoveElevator(elevator, Constants.Elevator.SAFE));

        operatorController.rightTrigger().whileTrue(new RunAlgaeIntake(algaeMechanism, Constants.AlgaeMechanism.ALGAE_INTAKE_SPEED));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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

        //vision
        joystick.rightBumper().whileTrue(new CenterAlign(drivetrain, vision));
        joystick.rightTrigger().whileTrue(new OffsetAlign(drivetrain, vision));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
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
