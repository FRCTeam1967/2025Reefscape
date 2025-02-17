// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignLED extends Command {
  Vision vision;
  LEDSubsystem ledSubsystem;
  private NetworkTable limelightTable;

  /** Creates a new AlignLED. */
  public AlignLED(Vision vision, LEDSubsystem ledSubsystem) {
    this.vision = vision;
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, ledSubsystem);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
  }
  
    
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  public int getTv(){
    return (int) limelightTable.getEntry("tv").getInteger(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(getTv() == 0){
      ledSubsystem.black();
    }
    else if (vision.getOffset() < 3.0 && vision.getOffset() >= -3.0 ) {
      ledSubsystem.green();

    } else {
      ledSubsystem.red();
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
