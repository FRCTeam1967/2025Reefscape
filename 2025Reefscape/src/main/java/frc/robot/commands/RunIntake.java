// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class RunIntake extends Command {
   private final Intake intake;
   private final double speed;

   public RunIntake(Intake intake, double speed) {
      this.intake = intake;
      this.speed = speed;
      addRequirements(intake);
   }

   public void execute() {
      intake.setMotors(speed);
   }

   public void end(boolean interrupted) {
      intake.stopMotors();
   }

   public boolean isFinished() {
    return intake.isBroken();
   }
}
