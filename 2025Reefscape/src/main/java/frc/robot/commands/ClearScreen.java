package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class ClearScreen extends Command {
    private final LEDSubsystem led;

    public ClearScreen(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    public void initialize() {
    }
    
    public void execute() {
      led.black();
    }
    public void end(boolean interrupted) {
      led.black();
    }

    public boolean isFinished() {
        return false;
    }
}
