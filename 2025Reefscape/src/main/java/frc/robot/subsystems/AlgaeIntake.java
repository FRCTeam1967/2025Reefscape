// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private TalonFX motor;
  
  /** Creates a new AlgaeIntake. 
   * @param motorID
  */
  public AlgaeIntake() {
      motor = new TalonFX(Constants.Algae.INTAKE_ID);
      var talonAlgaeIntakeConfigs = new TalonFXConfiguration();

      var slot0Configs = talonAlgaeIntakeConfigs.Slot0;
      slot0Configs.kS = Constants.Algae.INTAKE_kS; 
      slot0Configs.kV = Constants.Algae.INTAKE_kV; 
      slot0Configs.kA = Constants.Algae.INTAKE_kA; 
      slot0Configs.kP = Constants.Algae.INTAKE_kP; 
      slot0Configs.kI = Constants.Algae.INTAKE_kI;
      slot0Configs.kD = Constants.Algae.INTAKE_kD; 
  
      // set Motion Magic settings
      var motionMagicConfigs = talonAlgaeIntakeConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Algae.INTAKE_CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Algae.INTAKE_ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Algae.INTAKE_JERK;

      talonAlgaeIntakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      motor.setNeutralMode(NeutralModeValue.Brake);
      talonAlgaeIntakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      motor.getConfigurator().apply(talonAlgaeIntakeConfigs);

      
  }
  /** Runs intake
   * @param speed
   */
  public void runIntake(double speed){
    motor.set(speed);
 }


  public void setVelocity(double velocity) {
      //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

      motor.setControl(request);

  }
 /** Stops intake
  */

  public void stopIntake(){
    motor.stopMotor();
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
