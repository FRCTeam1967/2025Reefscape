/* package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class CenterAlignLED extends Command{private final Vision vision; 
    private final LEDSubsystem led;
    private final Timer stopwatch;
    public CenterAlignLED(Vision vision, LEDSubsystem led) {
        this.vision = vision;
        this.led = led;
        stopwatch = new Timer();
        addRequirements(vision, led);
    }

    public void execute() {
        if(vision.getIsInRange()) {
            stopwatch.reset(); 
            stopwatch.start();
            while (stopwatch.get() < 5) {
                led.scrollingRainbow();
            }
        }
    }
    public void end() {
    }

    public boolean isFinished() {
        return stopwatch.get() >= 4;
    }
}
*/
