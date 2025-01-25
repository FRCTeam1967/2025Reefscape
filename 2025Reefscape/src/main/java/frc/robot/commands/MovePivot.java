// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Pivot;

public class MovePivot extends Command {
   private Pivot pivot;
   private double targetPosition;

   public MovePivot(Pivot pivot, double targetPosition ) {
      this.pivot = pivot;
      this.targetPosition = targetPosition;
      addRequirements(this.pivot);
   }

   public void initialize() {
   }

   public void execute() {
      pivot.moveTo(targetPosition);
      pivot.checkLimit();
   }

   public boolean isFinished() {
      return pivot.isReached();
   }

   public void end(boolean interrupted) {

   }
}







