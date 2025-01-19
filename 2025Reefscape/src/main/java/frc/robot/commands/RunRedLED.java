package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class RunRedLED extends Command {
    private final LEDSubsystem led;

    public RunRedLED(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    public void initialize() {
        led.red();
    }
    
    public void execute() {
      
    }
    public void end(boolean interrupted) {
      led.black();
    }

    public boolean isFinished() {
        return false;
    }
}