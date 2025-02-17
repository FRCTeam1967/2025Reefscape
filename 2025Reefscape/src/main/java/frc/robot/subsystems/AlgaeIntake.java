// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private TalonFX motor;
  
  /** Creates a new AlgaeIntake. 
   * @param motorID
  */
  public AlgaeIntake() {
      motor = new TalonFX(Constants.AlgaeMechanism.INTAKE_ID);
      var talonAlgaeIntakeConfigs = new TalonFXConfiguration();
      
      talonAlgaeIntakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      motor.getConfigurator().apply(talonAlgaeIntakeConfigs);
  }
  /** Runs intake
   * @param speed
   */
  public void runIntake(double speed){
    motor.set(speed);
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
