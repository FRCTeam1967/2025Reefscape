// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  private TalonFX motor;

  /** Creates a new Intake. */
  public Intake() {
    motor = new TalonFX(Constants.Intake.MOTOR_ID);
  }

  public void runMotor(double speed){
    motor.set(speed);
  }

  public void stopMotor(){
    motor.set(0.0);
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
