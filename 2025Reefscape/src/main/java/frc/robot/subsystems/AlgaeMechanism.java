// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase {
   private TalonFX algaePivot;
   private TalonFX algaeIntake;
   //private CANcoder absEncoder; 
   private double revsToMove;
   private ShuffleboardTab tab;

   public AlgaeMechanism() {
      //absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID); 
      tab = Shuffleboard.getTab("match");

      //pivot instantiations
      algaePivot = new TalonFX(Constants.AlgaeMechanism.PIVOT_ID);
      var talonFXConfigs = new TalonFXConfiguration();

      //set slot configs
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.AlgaeMechanism.kS; 
      slot0Configs.kV = Constants.AlgaeMechanism.kV;
      slot0Configs.kA = Constants.AlgaeMechanism.kA;
      slot0Configs.kP = Constants.AlgaeMechanism.kP;
      slot0Configs.kI = Constants.AlgaeMechanism.kI;
      slot0Configs.kD = Constants.AlgaeMechanism.kD;

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.AlgaeMechanism.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.AlgaeMechanism.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.AlgaeMechanism.JERK;

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      algaePivot.getConfigurator().apply(talonFXConfigs);

      resetEncoders(); 
      algaePivot.setNeutralMode(NeutralModeValue.Brake);

      //intake instantiations
      algaeIntake = new TalonFX(Constants.AlgaeMechanism.INTAKE_ID);
      var talonAlgaeIntakeConfigs = new TalonFXConfiguration();
      
      talonAlgaeIntakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      algaeIntake.getConfigurator().apply(talonAlgaeIntakeConfigs);
   }

   //pivot methods
   public void stop() {
      algaePivot.stopMotor();
   }

   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.AlgaeMechanism.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      algaePivot.setControl(request);
   }

   public void resetEncoders() { 
      algaePivot.setPosition(0);
   }

   public boolean isReached() {
      return Math.abs(((algaePivot.getRotorPosition().getValueAsDouble()/Constants.AlgaeMechanism.GEAR_RATIO)*360) - (revsToMove*360)) < 5.0;
   }

   //intake methods
   public void runIntake(double speed){
      algaeIntake.set(speed);
   }

   public void stopIntake(){
      algaeIntake.stopMotor();
   }

   public void periodic() {
      tab.addNumber("Pivot Rel Position Degrees", () -> ((algaePivot.getRotorPosition().getValueAsDouble()/Constants.AlgaeMechanism.GEAR_RATIO)*360));
      tab.addBoolean("Pivot At Target", () -> isReached());
   }
}