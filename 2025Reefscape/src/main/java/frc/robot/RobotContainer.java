// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionUpdate;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
    public ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
    
    public VisionUpdate vision = new VisionUpdate(drivetrain);
    
    Optional<Alliance> ally = DriverStation.getAlliance(); 

    public RobotContainer() {
        //drivetrain.getPigeon2().setYaw(-23.06);
        configureBindings();
        configLLTab(limelightTab, fieldTab);        
    }

    private double limelight_aim_proportional() {        
        double kP = 0.02; //0.035
        double targetingAngularVelocity = 0.0; 
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.

        if (LimelightHelpers.getFiducialID("limelight-front") == 10 || LimelightHelpers.getFiducialID("limelight-front") == 26) {
            targetingAngularVelocity = (LimelightHelpers.getTX("limelight-front") * kP);
        }

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }
    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    private double limelight_range_proportional(){
        double kP = 0.01;
        double error = LimelightHelpers.getTY("limelight-front") - 10.0;
        double targetingForwardSpeed = error * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
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

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));//.seedFieldCentric()

        //POV buttons
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

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // hub alignment
        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(limelight_aim_proportional()) // Drive with targetAngularVelocity
            )
        );
        // // ranging
        // joystick.rightTrigger().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(limelight_range_proportional()) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // tower alignment using TX
        //joystick.leftTrigger().whileTrue(new AlignTowerTX(drivetrain, vision, false, true));

        // tower alignment using pose
        joystick.leftTrigger().whileTrue(new AlignTowerPose(drivetrain));


        //do both
        joystick.y().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(limelight_range_proportional()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(limelight_aim_proportional()) // Drive with targetAngularVelocity
            )
        );

        joystick.x().onTrue(new SequentialCommandGroup(
            // ROTATION2D IS IN **RADIANS!!!!**
            // SET YAW IS IN **DEGREES!!!!**
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI))),
                    new InstantCommand(() -> drivetrain.getPigeon2().setYaw(180.0)),
                    new InstantCommand(() -> drivetrain.getPigeon2().getYaw().waitForUpdate(0.1)),
                    new InstantCommand(() -> drivetrain.resetPose(new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), new Rotation2d(Math.PI))))        
                ),
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(new Rotation2d(0.0))),    
                    new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0.0)),
                    new InstantCommand(() -> drivetrain.getPigeon2().getYaw().waitForUpdate(0.1)),
                    new InstantCommand(() -> drivetrain.resetPose(new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), new Rotation2d(0))))
                ),
                () -> ally.get() == Alliance.Blue
            )
        ));
            
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command getAutonomousCommand() {
    //     return Commands.print("No autonomous command configured");
    // }

    public void configLLTab(ShuffleboardTab tab, ShuffleboardTab fieldTab) {
        HttpCamera httpCamera1 = new HttpCamera("limelight-front", "http://10.19.67.11:5801/"); //http://10.19.67.202:5801/
        CameraServer.addCamera(httpCamera1);
        tab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
        .withSize(3, 2);

        tab.addBoolean("LL isInRange", () -> getInRange(LimelightHelpers.getTY("limelight-front")))
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 1)
        .withSize(1, 1);

        tab.addBoolean("LL isAligned", () -> isAligned(LimelightHelpers.getTX("limelight-front")))
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 2)
        .withSize(1, 1);

        fieldTab.add("Field", CommandSwerveDrivetrain.m_field).withWidget(BuiltInWidgets.kField).withSize(8, 4);
    }

    public boolean getInRange(double position) {
        double target = 0.0;
        double threshold = 5.0;
        return position <=  (target + threshold) && position >= (target - threshold);
    }
    public boolean isAligned(double position) {
        double target = 0.0;
        double threshold = 5.0;
        return position <=  (target + threshold) && position >= (target - threshold);
    }
}