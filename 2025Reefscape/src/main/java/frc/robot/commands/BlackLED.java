// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlackLED extends Command {
  /** Creates a new BlackLED. */
  private final LEDSubsystem led;
  public BlackLED(LEDSubsystem led) {
    this.led = led;
    addRequirements(led);
    /** Use addRequirements() here to declare subsystem dependencies.*/
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {}

  /**Called every time the scheduler runs while the command is scheduled.*/
  @Override
  public void execute() {
    led.blackPattern();
  }

  /** Called once the command ends or is interrupted.*/
  @Override
  public void end(boolean interrupted) {}

  /**  Returns true when the command should end.*/
  @Override
  public boolean isFinished() {
    return false;
  }
}
