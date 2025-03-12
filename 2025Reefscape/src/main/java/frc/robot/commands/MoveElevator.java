// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
  private Elevator elevator;
  private double inches;
  private AlgaePivot algae;
  private double algaeAngle;
  //private boolean goingUp;
  /** Creates a new GoUp. */
  public MoveElevator(Elevator elevator, double inches, AlgaePivot algae) {
    this.elevator = elevator;
    this.inches = inches;
    this.algae = algae;
    //this.goingUp = goingUp;
    addRequirements(elevator);
    addRequirements(algae);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (inches == Constants.Elevator.CORAL_L4_HEIGHT){
      algaeAngle = Constants.Algae.L4_CORAL_SCORING_ANGLE;
    } else if (inches == Constants.Elevator.CORAL_L2_HEIGHT){
      algaeAngle = Constants.Algae.L2L3_CORAL_SCORING_ANGLE;
    } else if(inches == Constants.Elevator.CORAL_L3_HEIGHT) {
      algaeAngle = Constants.Algae.L2L3_CORAL_SCORING_ANGLE;
    } else {
      algaeAngle = Constants.Algae.SAFE;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.moveTo(inches);
    algae.moveTo(algaeAngle);
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
