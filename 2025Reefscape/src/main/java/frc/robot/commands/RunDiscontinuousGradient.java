package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;

public class RunDiscontinuousGradient extends Command {
    private final LEDSubsystem led;
    private final Timer stopwatch;

    public RunDiscontinuousGradient(LEDSubsystem led) {
      this.led = led;
      stopwatch = new Timer();
      addRequirements(led);
    }

    @Override
    public void initialize() {
      stopwatch.reset();
      stopwatch.start();

    }
    public void execute() {
      led.scrollDiscontinuousGradient();
    }

    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return stopwatch.get() >= 4;
    }
}