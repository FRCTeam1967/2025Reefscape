package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MaskPattern extends Command {
    private final LEDSubsystem led;

    public MaskPattern(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    public void initialize() {
    }
    
    public void execute() {
      led.maskRainbow();
      //Timer.delay(0.02);
    }
    public void end(boolean interrupted) {
      led.black();
    }
    public boolean isFinished() {
        return false;
    }
}