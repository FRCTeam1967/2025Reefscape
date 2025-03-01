// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunFunnel extends Command {
  private final Funnel funnel;
  private final double speed;
  //private final CoralIntake intake;

  /** Creates a new RunFunnel. */
  public RunFunnel(Funnel funnel, double speed) {
    this.funnel = funnel;
    //this.intake = intake;
    this.speed = speed;
    addRequirements(funnel);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnel.feedFunnel(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.stopFunnel();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return intake.isBroken();
    //return false;
  }
}
