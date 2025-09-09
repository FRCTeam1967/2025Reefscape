// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX topMotor; // Declare the top motor
  private TalonFX bottomMotor; // Declare the bottom motor

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new TalonFX(Constants.Shooter.TOP_SHOOTER_ID); // Set the top shooter ID to 1

    // Initialize the bottom motor in the same way:
    bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_SHOOTER_ID); // Use the BOTTOM_SHOOTER_ID value here
  }

  public void runShooter(double speed){ // Write the parameter the command needs to input
    topMotor.set(speed); // Write the same parameter here
    bottomMotor.set(speed); // Do it for the bottom motor too
  }

  public void stopShooter(){
    topMotor.stopMotor(); // Call the method which stops the motor from running
    bottomMotor.stopMotor(); // Call the same method for the bottom motor here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


