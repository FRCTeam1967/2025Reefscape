// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
    bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);
  }

  public void runMotors(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  public void stopMotors(){
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public double getTopShooterVelocity(){
    return (topMotor.getVelocity()).getValueAsDouble();
  }

  public double getBottomShooterVelocity(){
    return (bottomMotor.getVelocity()).getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
