// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw254;

public class MoveClaw extends Command {
   private Claw254 claw;
   private double targetPosition;

   public MoveClaw(Claw254 claw, double targetPosition ) {
      this.claw = claw;
      this.targetPosition = targetPosition;
      addRequirements(claw);
   }

   public void initialize() {
   }

   public void execute() {
      claw.moveTo(targetPosition);
   }

   public boolean isFinished() {
      return claw.isReached();
   }

   public void end(boolean interrupted) {

   }
}