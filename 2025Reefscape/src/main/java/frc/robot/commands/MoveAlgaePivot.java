// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

public class MoveAlgaePivot extends Command {
   private AlgaePivot pivot;
   private double targetPosition;

   public MoveAlgaePivot(AlgaePivot pivot, double targetPosition ) {
      this.pivot = pivot;
      this.targetPosition = targetPosition;
      addRequirements(pivot);
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