// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseBeamFunnel extends Command {
  public Funnel funnel;
  public final double speed;

  /** Creates a new ReverseBeamFeeder. */
  public ReverseBeamFunnel(Funnel funnel, double speed) {
    this.funnel = funnel;
    this.speed = speed;
    addRequirements(funnel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnel.feedFunnel(speed*-1);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.stopFunnel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !funnel.isBroken();
  }
}
