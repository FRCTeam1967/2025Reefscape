package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class Climb extends SubsystemBase {
   private TalonFX climbMotor;
   private TalonFXConfiguration config;
   public double revsToMove;

   public Climb() {
      climbMotor = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);

      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Climb.kS; 
      slot0Configs.kV = Constants.Climb.kV;
      slot0Configs.kA = Constants.Climb.kA;
      slot0Configs.kP = Constants.Climb.kP;
      slot0Configs.kI = Constants.Climb.kI;
      slot0Configs.kD = Constants.Climb.kD;

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Climb.JERK;

      // inverted value?
      talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      
      climbMotor.getConfigurator().apply(talonFXConfigs);

      climbMotor.setNeutralMode(NeutralModeValue.Brake);
      

      talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */

   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.Climb.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      climbMotor.setControl(request);
   }

   public void runClimb(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      climbMotor.setControl(request);
   }

   public void stopMotor() {
      climbMotor.stopMotor();
   }

   public void zeroEncoder() {
     climbMotor.setPosition(0);
   }

   /**Adds value to shuffleboard
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addDouble("Climb Rel Pos",()->(climbMotor.getRotorPosition().getValueAsDouble()))
      .withWidget(BuiltInWidgets.kTextView).withPosition(6, 0)
      .withSize(1, 1);

      tab.addBoolean("isReached",()->(climbMotor.getRotorPosition().getValueAsDouble() >= Constants.Climb.CLIMB_ENCODER_STOP_VAL))
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2)
      .withSize(1, 1);

   }

   @Override
   public void periodic() {    
     // This method will be called once per scheduler run                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
   }
}