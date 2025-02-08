// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class MoveElevator extends Command {
  private Elevator elevator;
  private double inches;
  //private boolean goingUp;
  /** Creates a new GoUp. */
  public MoveElevator(Elevator elevator, double inches) {
    this.elevator = elevator;
    this.inches = inches;
    //this.goingUp = goingUp;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.moveTo(inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.stopMotors();
    if (elevator.getSensor()) {
      elevator.stopMotors();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atHeight();
  }
}
