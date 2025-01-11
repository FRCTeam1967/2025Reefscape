// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Pivot;

public class MovePivot extends Command {
   private final Pivot m_pivot;
   private final double m_targetPosition;

   public MovePivot(Pivot pivot, double targetPosition) {
      this.m_pivot = pivot;
      this.m_targetPosition = targetPosition;
      this.addRequirements(new Subsystem[]{this.m_pivot});
   }

   public void initialize() {
      this.m_pivot.moveTo(this.m_targetPosition);
   }

   public void execute() {
   }

   public boolean isFinished() {
      return this.m_pivot.isReached();
   }

   public void end(boolean interrupted) {
      this.m_pivot.stop();
   }
}
