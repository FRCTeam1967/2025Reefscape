package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RunContinuousGradient extends Command {
    private final LEDSubsystem led;

    public RunContinuousGradient(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    @Override
    public void initialize() {
    }
    
    public void execute() {
      led.scrollingContinuousGradient();
    }
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}