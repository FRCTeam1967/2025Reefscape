// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GerryRig extends SubsystemBase {
  /** Creates a new GerryRig. */
  private TalonFX motor; 
  public GerryRig() {
    motor = new TalonFX(31);


  }
  public void runMotor(double speed){
    motor.set(speed);
  }
  public void stopMotor(){
    motor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
