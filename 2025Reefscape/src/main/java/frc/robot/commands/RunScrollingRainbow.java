
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
/**creates a class for RunScrollimgRainbow - runs a scrolling rainbow effect */
/**constructor takes an LEDSubsystem object */
public class RunScrollingRainbow extends Command {
    private final LEDSubsystem led;

    public RunScrollingRainbow(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    public void initialize() {
    
    }
    
    public void execute() {
      led.scrollingRainbow();
      
    }
    public void end(boolean interrupted) {
      led.blackPattern();
  
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


