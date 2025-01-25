// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMechanism;

public class MoveAlgaePivot extends Command {
   private AlgaeMechanism pivot;
   private double targetPosition;

   public MoveAlgaePivot(AlgaeMechanism pivot, double targetPosition ) {
      this.pivot = pivot;
      this.targetPosition = targetPosition;
      addRequirements(this.pivot);
   }

   public void initialize() {
   }

   public void execute() {
      pivot.moveTo(targetPosition);
   }

   public boolean isFinished() {
      return pivot.isReached();
   }

   public void end(boolean interrupted) {

   }
}