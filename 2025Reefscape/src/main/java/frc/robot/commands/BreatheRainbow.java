/* 

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class BreatheRainbow extends Command {
    private final LEDSubsystem led;

    public BreatheRainbow(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    @Override
    public void initialize() {
    }
    
    public void execute() {
      led.breatheGradient();
    }
    public void end(boolean interrupted) {
      led.black();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

*/