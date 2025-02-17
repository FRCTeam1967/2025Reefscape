package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class RunGreenLED extends Command {
    private final LEDSubsystem led;

    public RunGreenLED(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    public void initialize() {
    }
    
    public void execute() {
      led.green();
    }
    public void end(boolean interrupted) {
      led.black();
    }

    public boolean isFinished() {
        return false;
    }
}