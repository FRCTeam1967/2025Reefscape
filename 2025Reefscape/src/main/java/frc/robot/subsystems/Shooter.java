// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private SparkMax _________; // Declare the top motor
  private SparkMax _________; // Declare the bottom motor

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new SparkMax(Constants.Shooter.TOP_SHOOTER_ID, MotorType.kBrushless); // Set the top shooter ID to 1

    // Initialize the bottom motor in the same way:
    _________ = new _________(Constants.Shooter._________, MotorType.kBrushless); // Use the BOTTOM_SHOOTER_ID value here

    //Configurations
    SparkMaxConfig config = new SparkMaxConfig();
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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


