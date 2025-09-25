// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunLED extends Command {
  /** Creates a new RunLED. */
  public RunLED() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //call your methods from the LEDSubsystem here!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //call your methods from the LEDSubsystem here!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //remember to turn off your LED!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
