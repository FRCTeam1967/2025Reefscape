// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import dev.doglog.DogLog;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
      
  Optional<Alliance> ally = DriverStation.getAlliance(); 

  private static final double kP_translational = 1.5;
  private static final double kP_rotational = 1.7; //1.2
  private Transform2d difference = new Transform2d();
  private Pose2d towerPose;


  private final StructPublisher<Pose2d> towerPublisher;

  public AlignTowerPose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    towerPublisher = NetworkTableInstance.getDefault().getTable("alignment").getStructTopic("tower", Pose2d.struct).publish();  
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            towerPose = new Pose2d(15.56, 2.98, new Rotation2d(0));
            //DogLog.log("Tower Pose: ", towerPose);
        }
        if (ally.get() == Alliance.Blue) {
            towerPose = new Pose2d(0.84, 4.8, new Rotation2d(Math.PI));
            //DogLog.log("Tower Pose: ", towerPose);
        }
    }

    towerPublisher.set(towerPose);
    DogLog.log("Tower Pose: ", towerPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Pose2d drivetrainPose = drivetrain.getPose();
    // 0.84, 4.8, 0

    difference = towerPose.minus(drivetrain.getPose());
    // 1.092, 4.61, 3.14

    if (ally.get() == Alliance.Red) {
        DogLog.log("Pose difference: ", difference);
        DogLog.log("pose diff x", difference.getX());
        DogLog.log("pose diff y", difference.getY());

        double xVelocity = MathUtil.clamp(difference.getX() * kP_translational, -MaxSpeed, MaxSpeed);
        DogLog.log("xVelocity: ", xVelocity);

        double yVelocity = MathUtil.clamp(difference.getY() * kP_translational, -MaxSpeed, MaxSpeed);
        DogLog.log("yVelocity: ", yVelocity);

        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate);
        DogLog.log("rotationalVelocity: ", rotationalVelocity);

        ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrain.getPigeon2().getRotation2d());
        DogLog.log("alignmentSpeed: ", alignmentSpeed);

        drivetrain.setControl(request.withSpeeds(alignmentSpeed));
        DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
        DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
        DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));
    }
    else {
          // Pose2d drivetrainPose = drivetrain.getPose();
          // // 0.84, 4.8, 0

          // difference = towerPose.minus(drivetrainPose);
          // // 0.252, 0.19, -3.14

        DogLog.log("Pose difference: ", difference); // // 0.252, 0.19, -3.14
        DogLog.log("pose diff x", difference.getX());
        DogLog.log("pose diff y", difference.getY());

        double xVelocity = MathUtil.clamp(-difference.getX() * kP_translational, -MaxSpeed, MaxSpeed); //-0.189
        DogLog.log("xVelocity: ", xVelocity);

        double yVelocity = MathUtil.clamp(-difference.getY() * kP_translational, -MaxSpeed, MaxSpeed); //-0.1425
        DogLog.log("yVelocity: ", yVelocity);

        double rotationalVelocity = MathUtil.clamp(difference.getRotation().getRadians() * kP_rotational, -MaxAngularRate, MaxAngularRate); // -2.355
        DogLog.log("rotationalVelocity: ", rotationalVelocity);

        ChassisSpeeds alignmentSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationalVelocity, drivetrain.getPigeon2().getRotation2d());
        DogLog.log("alignmentSpeed: ", alignmentSpeed);

        drivetrain.setControl(request.withSpeeds(alignmentSpeed));
        DogLog.log("x isFinished: ", Math.abs(difference.getX()) < 0.05);
        DogLog.log("y isFinished: ", Math.abs(difference.getY()) < 0.05);
        DogLog.log("rotation isFinished: ", Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(2));

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.setControl(request.withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(difference.getX()) < 0.05 &&
             Math.abs(difference.getY()) < 0.05 &&
             Math.abs(difference.getRotation().getRadians()) < Units.degreesToRadians(0.05);
  }
}