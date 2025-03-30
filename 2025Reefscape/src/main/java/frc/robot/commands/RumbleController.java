// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RumbleController extends Command {
  private CommandXboxController driverXbox;
  private CommandPS4Controller operatorXbox;
  
  /** Creates a new RumbleController. */
  public RumbleController(CommandXboxController driverXbox, CommandPS4Controller operatorXbox) {
    this.driverXbox = driverXbox;
    this.operatorXbox = operatorXbox;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.8);
    operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0.8);
  }

  @Override 
  public void end(boolean interrupted) {
    driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
    operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}