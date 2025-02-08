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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Pivot extends SubsystemBase {
   private TalonFX pivotMotor;
   private DigitalInput limitSwitch;
   //private CANcoder absEncoder; 
   public double revsToMove;
   
   //* Creates a new pivot */
   public Pivot() {
      //absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID); 
      pivotMotor = new TalonFX(Constants.Pivot.PIVOT_ID);
      limitSwitch = new DigitalInput(Constants.Pivot.SWITCH_ID);

      var talonFXConfigs = new TalonFXConfiguration();

      //* sets  slot configs */
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Pivot.kS; 
      slot0Configs.kV = Constants.Pivot.kV;
      slot0Configs.kA = Constants.Pivot.kA;
      slot0Configs.kP = Constants.Pivot.kP;
      slot0Configs.kI = Constants.Pivot.kI;
      slot0Configs.kD = Constants.Pivot.kD;


      //sets motion magic configurations
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK;

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      //applies motion magic configs
      pivotMotor.getConfigurator().apply(talonFXConfigs);

      resetEncoders(); 
      pivotMotor.setNeutralMode(NeutralModeValue.Brake);
   }

   /** Stops the pivot motor */
   public void stop() {
      pivotMotor.stopMotor();
   }

   /** creates a Motion Magic request, sets target position to revsToMove */
   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.Pivot.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      pivotMotor.setControl(request);
   }
   
   /** checks if pivot has surpassed hard stop
    * if limit has been reached, sets pivot motor speed to 0 */
   public void checkLimit(){
      if (limitSwitch.get()){
         pivotMotor.set(0);
      }
   }

   /** zeroes the motor (sets its position to 0) */
   public void resetEncoders() { 
      pivotMotor.setPosition(0);
   }

   /** checks if the position the motor is at is within error threshold of the end goal */
   public boolean isReached() {
      return Math.abs(((pivotMotor.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360) - (revsToMove*360)) < 5.0;
   }

   public void periodic() {
      //SmartDashboard.putNumber("Pivot Rel Position Degrees",(pivotMotor.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360);
      //SmartDashboard.putBoolean("Pivot At Target", isReached());
   }

   public void configDashboard(ShuffleboardTab tab) {
      tab.add("Pivot Rel Position Degrees",(pivotMotor.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360);
      tab.add("Pivot At Target", isReached());
   }
}