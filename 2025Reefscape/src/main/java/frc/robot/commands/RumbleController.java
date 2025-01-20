/* 

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleController extends Command {
  private CommandXboxController driverXbox, operatorXbox;
  
  public RumbleController(CommandXboxController driverXbox, CommandXboxController operatorXbox) {
    this.driverXbox = driverXbox;
    this.operatorXbox = operatorXbox;
  }

  @Override
  public void initialize() {

  }

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

*/

