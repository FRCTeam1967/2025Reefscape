// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.commands.BlackLED;
import frc.robot.commands.BlinkGradient;
import frc.robot.commands.MovePivot;
import frc.robot.commands.RumbleController;
import frc.robot.commands.RunContinuousGradient;
import frc.robot.commands.RunDiscontinuousGradient;
import frc.robot.commands.RunRedLED;
import frc.robot.commands.RunScrollingRainbow;
import frc.robot.Constants;
import frc.robot.Constants.Xbox;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class RobotContainer {
   private final Intake intake = new Intake();
   private final Pivot pivot = new Pivot();
   private final LEDSubsystem led = new LEDSubsystem();
   private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
   private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);

   public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
   
   public RobotContainer() {
      this.configureBindings();
      pivot.configDashboard(matchTab);
   }

   private void configureBindings() {
      /** Set the default command to turn the strip off, otherwise the last colors written by the last command to run will continue to be displayed.*/
      led.setDefaultCommand(new BlackLED(led));
      operatorController.y().onTrue(new MovePivot(pivot, Constants.Pivot.MIDDLE));
      operatorController.x().onTrue(new MovePivot(pivot, Constants.Pivot.L2));
      operatorController.a().onTrue(new MovePivot(pivot, Constants.Pivot.SAFE));

      operatorController.rightTrigger().whileTrue(new RunIntake(intake, led, Constants.Intake.SLOW));
      operatorController.leftTrigger().whileTrue(new RunIntake(intake, led, Constants.Intake.REVERSE_SLOW));
      //operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new RunIntake(intake, led, Constants.Intake.HIGH));


      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunIntake(intake, led, Constants.Intake.HIGH), new RumbleController(driverController, operatorController).withTimeout(2)));
      operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunIntake(intake, led, Constants.Intake.SLOW), new RumbleController(driverController, operatorController).withTimeout(2)));

      operatorController.b().onTrue(new RunDiscontinuousGradient(led));
   }
}
