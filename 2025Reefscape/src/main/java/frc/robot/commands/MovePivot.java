// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/**define class MovePivot */
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
/**
 moves the pivot to the target position
 checks if it has passed the hardstop checkpoint using checkLimit
 */
   public void execute() {
      pivot.moveTo(targetPosition);
      pivot.checkLimit();
   }
/**
 * checks if the pivot has reached target position
 * */
   public boolean isFinished() {
      return pivot.isReached();
   }

   public void end(boolean interrupted) {

   }
}







