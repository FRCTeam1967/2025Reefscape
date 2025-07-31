// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake254;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunRollers extends Command {
  private Intake254 intake;
  private double sideSpeed;
  private double topSpeed;

  /** Creates a new RunAlgaeIntake. */
  public RunRollers(Intake254 intake, double sideSpeed, double topSpeed) {
    this.intake = intake;
    this.sideSpeed = sideSpeed;
    this.topSpeed = topSpeed;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runSideIntake(sideSpeed);
    intake.runTopIntake(topSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopSideIntake();
    intake.stopTopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}