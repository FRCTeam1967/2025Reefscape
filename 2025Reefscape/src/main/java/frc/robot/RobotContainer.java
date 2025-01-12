// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.MovePivot;
import frc.robot.commands.RumbleController;
import frc.robot.Constants;
import frc.robot.Constants.Xbox;


public class RobotContainer {
   private final Intake intake = new Intake();
   private final Pivot pivot = new Pivot();
   private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
   private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);

   public RobotContainer() {
      this.configureBindings();
   }

   private void configureBindings() {
      operatorController.y().onTrue(new MovePivot(pivot, Constants.Pivot.UP));
      operatorController.x().onTrue(new MovePivot(pivot, Constants.Pivot.MIDDLE));
      operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new RunIntake(intake, Constants.Intake.SLOW));
      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new RunIntake(intake, Constants.Intake.HIGH));


      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunIntake(intake, Constants.Intake.HIGH), new RumbleController(driverController, operatorController).withTimeout(2)));
      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunIntake(intake, Constants.Intake.SLOW), new RumbleController(driverController, operatorController).withTimeout(2)));
   }
}
