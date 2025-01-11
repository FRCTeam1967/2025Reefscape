// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class RunIntake extends Command {
   private final Intake intake;
   private final DoubleSupplier speedSupplier;

   public RunIntake(Intake intake, DoubleSupplier speedSupplier) {
      this.intake = intake;
      this.speedSupplier = speedSupplier;
      this.addRequirements(new Subsystem[]{intake});
   }

   public void execute() {
      this.intake.setMotors(this.speedSupplier.getAsDouble());
   }

   public void end(boolean interrupted) {
      this.intake.stopMotors();
   }

   public boolean isFinished() {
      return false;
   }
}
