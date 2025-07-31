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

public class Intake254 extends SubsystemBase {
  private TalonFX sideRoller;
  private TalonFX topRoller;
  
  /** Creates a new Intake254. 
   * @param motorID
  */
  public Intake254() {
      sideRoller = new TalonFX(Constants.Intake.SIDE_ROLLER_ID);
      topRoller = new TalonFX(Constants.Intake.TOP_ROLLER_ID);
      var talonIntakeConfigs = new TalonFXConfiguration();

      var slot0Configs = talonIntakeConfigs.Slot0;
      slot0Configs.kS = Constants.Intake.INTAKE_kS; 
      slot0Configs.kV = Constants.Intake.INTAKE_kV; 
      slot0Configs.kA = Constants.Intake.INTAKE_kA; 
      slot0Configs.kP = Constants.Intake.INTAKE_kP; 
      slot0Configs.kI = Constants.Intake.INTAKE_kI;
      slot0Configs.kD = Constants.Intake.INTAKE_kD; 
  
      // set Motion Magic settings
      var motionMagicConfigs = talonIntakeConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.INTAKE_CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.INTAKE_ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Intake.INTAKE_JERK;

      talonIntakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      sideRoller.setNeutralMode(NeutralModeValue.Brake);
      topRoller.setNeutralMode(NeutralModeValue.Brake);
      talonIntakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      sideRoller.getConfigurator().apply(talonIntakeConfigs);
      topRoller.getConfigurator().apply(talonIntakeConfigs);

      
  }
  /** Runs side intake
   * @param speed
   */
  public void runSideIntake(double speed){
    sideRoller.set(speed);
  }

  /** Runs top intake
   * @param speed
   */
  public void runTopIntake(double speed){
    topRoller.set(speed);
  }

  public void setSideVelocity(double velocity) {
    //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

    sideRoller.setControl(request);
  }

  public void setTopVelocity(double velocity) {
    //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

    topRoller.setControl(request);
  }

  /** Stops side rollers
  */
  public void stopSideIntake(){
    sideRoller.stopMotor();
  }

  /** Stops top rollers
  */
  public void stopTopIntake(){
    topRoller.stopMotor();
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
