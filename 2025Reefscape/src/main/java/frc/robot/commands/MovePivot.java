
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class MovePivot extends Command {
   private Pivot pivot;
   private double targetPosition;

   public MovePivot(Pivot pivot, double targetPosition ) {
      this.pivot = pivot;
      this.targetPosition = targetPosition;
      addRequirements(pivot);
   }

   public void initialize() {
   }

   public void execute() {
      pivot.moveTo(targetPosition);
   }

   public void end(boolean interrupted) {
   }  

   public boolean isFinished() {
      return pivot.isReached();
   }
}