// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;


public class RunIntake extends Command {
   private final Intake intake;
   private final double speed;
   private final LEDSubsystem led;
   private final Timer stopwatch;

   public RunIntake(Intake intake, LEDSubsystem led, double speed) {
      this.intake = intake;
      this.speed = speed;
      this.led = led;
      stopwatch = new Timer();
      addRequirements(intake);
   }
/** sets the intake motors to a certain speed defined in SLOW or HIGH in Constants */
   public void execute() {
      intake.setMotor(speed);
   }
/** stops the intake motors and sets the LED to red when the command is done */
   public void end(boolean interrupted) {
      intake.stopMotor();
      stopwatch.reset();
      stopwatch.start();
      while (stopwatch.get() < 3) {
         led.scrollingRed();
     }
   }
/** checks if beam in beam break sensor has been broken*/
   public boolean isFinished() {
      return intake.isBroken();
   }
}
