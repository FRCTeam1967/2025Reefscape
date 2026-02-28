// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Vision;


import frc.robot.commands.AlignLED;
import frc.robot.commands.ClearScreen;
import frc.robot.commands.RunRedLED;
import frc.robot.commands.RunGreenLED;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final LEDSubsystem led = new LEDSubsystem();
  //private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight tab");
  private final Vision vision = new Vision("limelight");

  public RobotContainer() {
    vision.configDashboard(limelightTab);
    configureBindings();
  }

 public Command alignLED() {      
      Command commandGroup;
      if (vision.alignAngleX()){
        commandGroup = new SequentialCommandGroup (
          new RunGreenLED(led), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
      } 
      else if (vision.getOffset() < -Constants.Vision.DEGREE_ERROR){
        commandGroup = new SequentialCommandGroup (
          new RunRedLED(led), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
       
      } else if (vision.getOffset() > Constants.Vision.DEGREE_ERROR){
        commandGroup = new SequentialCommandGroup (
          new RunRedLED(led), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
        
      } else{
        commandGroup = new SequentialCommandGroup (
          new ClearScreen(led), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
      }
    return commandGroup;
  }

  public void LEDscheduler (){
    CommandScheduler.getInstance().schedule(alignLED());
 }

  private void configureBindings() {
   //m_controller.leftTrigger().whileTrue(new AlignLED(vision, led));
  }

  public Command getAutonomousCommand() {
    return Commands.print("no auto config");
  }

  public Command getTeleopCommand() {
    return new AlignLED(vision, led);
  }

}