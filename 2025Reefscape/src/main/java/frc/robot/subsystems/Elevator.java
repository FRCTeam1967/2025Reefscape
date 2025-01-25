// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private double rotations;
  private DigitalInput sensor;
  private boolean atZero;
  //private TalonFX rightMotor;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_IDX, "Canivore");
    rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_IDX, "Canivore");
    sensor = new DigitalInput(Constants.Elevator.SENSOR_ID);
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
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
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftMotor.getConfigurator().apply(talonFXConfigs);
    rightMotor.getConfigurator().apply(talonFXConfigs);

    resetEncoders();
    setBrakeMode();

  }

  public void stopMotors(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();

  }

  public void moveTo(double inches){
    rotations = inches*(Constants.Elevator.GEAR_RATIO/Constants.Elevator.SPROCKET_PITCH_CIRCUMFERENCE);
    MotionMagicVoltage request = new MotionMagicVoltage(rotations).withFeedForward(Constants.Elevator.FEED_FORWARD);
    leftMotor.setControl(request); 
    rightMotor.setControl(request);       
      
  }

  public boolean atHeight(){
    double currentPosition = (leftMotor.getRotorPosition().getValueAsDouble() + rightMotor.getRotorPosition().getValueAsDouble())/2;
    double targetPosition = rotations;
    double error = Math.abs(targetPosition - currentPosition);
    return (error < Constants.Elevator.ERROR_THRESHOLD);
  }

  public void resetEncoders() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);

  }
  public void setBrakeMode() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    double height = ((rightMotor.getRotorPosition().getValueAsDouble() + leftMotor.getRotorPosition().getValueAsDouble())/2)/(Constants.Elevator.GEAR_RATIO/Constants.Elevator.SPROCKET_PITCH_CIRCUMFERENCE);
    SmartDashboard.putNumber("elevator height in inches", height);
    SmartDashboard.putNumber("elevator height in revs", (rightMotor.getRotorPosition().getValueAsDouble() + leftMotor.getRotorPosition().getValueAsDouble())/2);
    // This method will be called once per scheduler run
  }
}