// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX _________; // Declare the top motor
  private TalonFX _________; // Declare the bottom motor

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new TalonFX(Constants.Shooter.TOP_SHOOTER_ID); // Set the top shooter ID to 1

    // Initialize the bottom motor in the same way:
    _________ = new _________(Constants.Shooter._________); // Use the BOTTOM_SHOOTER_ID value here
  }

  public void runShooter(double _________){ // Write the parameter the command needs to input
    topMotor.set(_________); // Write the same parameter here
    bottomMotor.set(_________); // Do it for the bottom motor too
  }

  public void stopShooter(){
    _________._________(); // Call the method which stops the motor from running
    _________._________(); // Call the same method for the bottom motor here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


