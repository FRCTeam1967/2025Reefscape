// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StageCoral extends Command {
  /** Creates a new StageCoral. */
  private final CoralIntake intake;
  private final double rotations;
  private double initialPos;
  private double finalPos;

  public StageCoral(CoralIntake intake, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.rotations = rotations;
    // MS: Shouldn't this declare a requirement on the intake?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // MS: This obviously makes it hard to tell whether it ran after it has run the first time. If that's important, you could have a static
    // integer in the command, and increment it and publish it to the dashboard here. Have it start at zero, and then you can just make sure
    // it increments every time you want to see if it ran.
    SmartDashboard.putString("did we run?", "yes!");
    initialPos = intake.getRotorPos();
    finalPos = initialPos + rotations;
    //intake.zeroEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.moveTo(finalPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.zeroEncoder();
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = intake.getRotorPos();
    double error = Math.abs(finalPos - currentPosition);
    return (error < Constants.CoralIntake.ERROR_THRESHOLD);
  }
}
