// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeScoring extends Command {
  private Elevator elevator;
  private AlgaePivot algaePivot;
  private AlgaeIntake algaeIntake;
  /** Creates a new BargeScoring. */
  public BargeScoring( AlgaePivot algaePivot, Elevator elevator, AlgaeIntake algaeIntake) {
    this.elevator = elevator;
    this.algaePivot = algaePivot;
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaePivot.moveTo(Constants.Algae.BARGE_SCORING_ANGLE);
    elevator.moveTo(Constants.Elevator.MAX_HEIGHT);
    if ( elevator.getHeight() > Constants.Elevator.BARGE_SCORING_HEIGHT) {
      algaeIntake.runIntake(Constants.Algae.ALGAE_OUTTAKE_SPEED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atHeight();
  }
}
