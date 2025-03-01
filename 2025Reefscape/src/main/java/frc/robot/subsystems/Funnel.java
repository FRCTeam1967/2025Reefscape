// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */
  private TalonFX funnelMotor;

  public Funnel() {
    funnelMotor = new TalonFX(Constants.Funnel.FUNNEL_MOTOR_ID);
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted  = InvertedValue.CounterClockwise_Positive;
    funnelMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void feedFunnel(double speed){
    funnelMotor.set(speed);
    
  }

  public void stopFunnel(){
    funnelMotor.stopMotor();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

