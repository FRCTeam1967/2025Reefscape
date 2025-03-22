// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.lang.constant.Constable;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake;

public class RunCoralIntake extends Command {
   public final CoralIntake intake;
   private final double speed;
   //private final LEDSubsystem led;
   //private final Timer stopwatch;

   public RunCoralIntake(CoralIntake intake, double speed) {
      this.intake = intake;
      this.speed = speed;
      //this.led = led;
      //stopwatch = new Timer();
      addRequirements(intake);
   }

   @Override
  public void initialize() {
   //  intake.zeroEncoder();
  }
/** sets the intake motors to a certain speed defined in SLOW or HIGH in Constants */
   public void execute() {
      intake.setVelocity(speed);
   }
/** stops the intake motors and sets the LED to red when the command is done */
   public void end(boolean interrupted) {
      intake.stopMotor();
      //new WaitCommand(3);
      //intake.zeroEncoder();
      //new StageCoral(intake, speed);
      //stopwatch.reset();
      //stopwatch.start();
      //while (stopwatch.get() < 3) {
         //led.scrollingRed();
     }
/** checks if beam in beam break sensor has been broken*/
   public boolean isFinished() {
      return intake.intakeBroken();
   }
}