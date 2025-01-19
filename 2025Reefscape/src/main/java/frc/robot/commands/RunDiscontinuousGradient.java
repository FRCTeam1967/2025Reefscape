package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RunDiscontinuousGradient extends Command {
    private final LEDSubsystem led;

    public RunDiscontinuousGradient(LEDSubsystem led) {
      this.led = led;
      addRequirements(led);
    }

    @Override
    public void initialize() {
      
    }
    
    public void execute() {
      led.scrollDiscontinuousGradient();
    }
    public void end(boolean interrupted) {
      led.black();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}