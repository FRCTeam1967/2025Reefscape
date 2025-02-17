// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaePivot extends SubsystemBase {
   private TalonFX algaePivot;
   private CANcoder absEncoder; 
   public double revsToMove;
   private boolean synced = false;

   /** 
    * 
    */
   public AlgaePivot() {
      absEncoder = new CANcoder(Constants.AlgaeMechanism.ENCODER_ID); 

      //pivot instantiations
      algaePivot = new TalonFX(Constants.AlgaeMechanism.PIVOT_ID);
      var talonFXConfigs = new TalonFXConfiguration();
     
      CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();

      ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      ccdConfigs.MagnetSensor.MagnetOffset = -0.29900390625;

      //set slot configs
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.AlgaeMechanism.kS; 
      slot0Configs.kV = Constants.AlgaeMechanism.kV;
      slot0Configs.kA = Constants.AlgaeMechanism.kA;
      slot0Configs.kP = Constants.AlgaeMechanism.kP;
      slot0Configs.kI = Constants.AlgaeMechanism.kI;
      slot0Configs.kD = Constants.AlgaeMechanism.kD;

      talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.AlgaeMechanism.CURRENT_LIMIT));

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.AlgaeMechanism.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.AlgaeMechanism.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.AlgaeMechanism.JERK;

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      algaePivot.getConfigurator().apply(talonFXConfigs);
      absEncoder.getConfigurator().apply(ccdConfigs);

      //resetEncoders(); 
      algaePivot.setNeutralMode(NeutralModeValue.Brake);
   }

   //pivot methods
   /** Stops pivot motor */
   public void stop() {
      algaePivot.stopMotor();
   }

   /** Sets param to MotionMagic goal and gives request to pivot motor
    * @param revolutions - goal height
    */
   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.AlgaeMechanism.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      algaePivot.setControl(request);
   }
   /** If not already synced, sets relative encoder equal to absolute encoder and switches boolean value */
   public void setReltoAbs(){
      if (!synced){
         algaePivot.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.AlgaeMechanism.GEAR_RATIO);
         synced = !synced;
      }
      //algaePivot.setPosition(0);
      // algaePivot.getConfigurator().setPosition(absEncoder.getAbsolutePosition().getValueAsDouble());
      // moveTo(algaePivot.getRotorPosition().getValueAsDouble());
   }
   /** Resets encoders/ sets the position to 0 */
   public void resetEncoders() { 
      algaePivot.setPosition(0);
   }
   /**  Checks whether or not the pivot has reached target position
    * @return boolean- true if goal is reached and false otherwise
    */
   public boolean isReached() {
         //return Math.abs((absEncoder.getAbsolutePosition().getValueAsDouble()*360) - ((revsToMove/Constants.AlgaeMechanism.GEAR_RATIO)*360)) < 5.0;
         return Math.abs(((algaePivot.getRotorPosition().getValueAsDouble()/Constants.AlgaeMechanism.GEAR_RATIO)*360) - ((revsToMove/Constants.AlgaeMechanism.GEAR_RATIO)*360)) < 5.0;
   
      }
   
   public void configDashboard(ShuffleboardTab tab) {
      tab.addNumber("Algae Pivot Rel Position Degrees",() -> ((algaePivot.getRotorPosition().getValueAsDouble()/Constants.AlgaeMechanism.GEAR_RATIO)*360));
      tab.addBoolean("Algae Pivot At Target", () -> isReached());
      tab.addNumber("Algae Abs Encoder", () -> absEncoder.getAbsolutePosition().getValueAsDouble());
   }

   //intake methods
   public void periodic() {
   }
}