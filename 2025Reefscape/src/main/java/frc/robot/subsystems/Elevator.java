// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX motorOne;
  private TalonFX motorTwo;
  private TalonFXConfiguration talonFXConfigs;
  private double rotations;

  /** Creates a new Elevator. */
  public Elevator() {
    motorOne = new TalonFX(Constants.Elevator.MOTOR_ID_ONE);
    motorTwo = new TalonFX(Constants.Elevator.MOTOR_ID_TWO);
    rotations = 0.0;

    talonFXConfigs = new TalonFXConfiguration();

    //motion magic configs
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Elevator.kS; 
    slot0Configs.kV = Constants.Elevator.kV; 
    slot0Configs.kA = Constants.Elevator.kA; 
    slot0Configs.kP = Constants.Elevator.kP; 
    slot0Configs.kI = Constants.Elevator.kI;
    slot0Configs.kD = Constants.Elevator.kD;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Elevator.JERK;

    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorOne.getConfigurator().apply(talonFXConfigs);
    motorTwo.getConfigurator().apply(talonFXConfigs);

  }

  public void moveTo(double rotations) {
    MotionMagicVoltage request = new MotionMagicVoltage(rotations);
    motorOne.setControl(request);
    motorTwo.setControl(request);
  }

  public double getPosition() {
    double average = (motorOne.getRotorPosition().getValueAsDouble() + motorTwo.getRotorPosition().getValueAsDouble())/2.0;
    return average;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
