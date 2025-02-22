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

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
      absEncoder = new CANcoder(Constants.Algae.ENCODER_ID); 

      //pivot instantiations
      algaePivot = new TalonFX(Constants.Algae.PIVOT_ID);
      var talonFXConfigs = new TalonFXConfiguration();
     
      CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();

      ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      ccdConfigs.MagnetSensor.MagnetOffset = -0.29900390625;

      //set slot configs
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Algae.kS; 
      slot0Configs.kV = Constants.Algae.kV;
      slot0Configs.kA = Constants.Algae.kA;
      slot0Configs.kP = Constants.Algae.kP;
      slot0Configs.kI = Constants.Algae.kI;
      slot0Configs.kD = Constants.Algae.kD;

      talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Algae.CURRENT_LIMIT));

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Algae.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Algae.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Algae.JERK;

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
      revsToMove = revolutions*(Constants.Algae.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      algaePivot.setControl(request);
   }
   /** If not already synced, sets relative encoder equal to absolute encoder and switches boolean value */
   public void setReltoAbs(){
      //if (!synced){
         algaePivot.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble()*Constants.Algae.GEAR_RATIO);
         //synced = !synced;
      //}
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
         //return Math.abs((absEncoder.getAbsolutePosition().getValueAsDouble()*360) - ((revsToMove/Constants.Algae.GEAR_RATIO)*360)) < 5.0;
         return Math.abs(((algaePivot.getRotorPosition().getValueAsDouble()/Constants.Algae.GEAR_RATIO)*360) - ((revsToMove/Constants.Algae.GEAR_RATIO)*360)) < 5.0;
   
      }
   
   public void configDashboard(ShuffleboardTab tab) {
      tab.addNumber("APivotRelPosDeg",() -> ((algaePivot.getRotorPosition()
      .getValueAsDouble()/Constants.Algae.GEAR_RATIO)*360))
      .withWidget(BuiltInWidgets.kTextView).withPosition(3, 0)
      .withSize(1, 1);
      tab.addBoolean("APivot Reached", () -> isReached())
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 0)
      .withSize(1, 1);
      tab.addNumber("Algae Abs Pos", () -> absEncoder.getAbsolutePosition().getValueAsDouble())
      .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0)
      .withSize(1, 1);
   }

   //intake methods
   public void periodic() {
   }
}