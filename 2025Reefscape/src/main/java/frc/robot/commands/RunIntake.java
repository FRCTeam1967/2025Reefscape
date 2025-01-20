// Source code is decompiled from a .class file using FernFlower decompiler.
/* 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;

public class RunIntake extends Command {
   private final Intake intake;
   private final double speed;
   private final LEDSubsystem led;

   public RunIntake(Intake intake, LEDSubsystem led, double speed) {
      this.intake = intake;
      this.speed = speed;
      this.led = led;
      addRequirements(intake);
   }

   public void execute() {
      intake.setMotors(speed);
      if (intake.isBroken()){
         led.red();
         //led.continuousGradient();
         //led.scrollingRainbow();
      }
   }

   public void end(boolean interrupted) {
      intake.stopMotors();
   }

   public boolean isFinished() {
      return intake.isBroken();
   }
}

*/

