// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
   private final Intake m_intake = new Intake();
   private final Pivot m_pivot = new Pivot();
   private final XboxController m_controller = new XboxController(0);

   public RobotContainer() {
      this.configureButtonBindings();
   }

   public void maintainPivotPosition() {
      this.m_pivot.setRelToAbs();
   }

   private void configureButtonBindings() {
      (new Trigger(() -> {
         return Math.abs(this.m_controller.getLeftY()) > 0.1;
      })).whileTrue(new RunIntake(this.m_intake, () -> {
         return this.m_controller.getLeftY();
      }));
   }
}
