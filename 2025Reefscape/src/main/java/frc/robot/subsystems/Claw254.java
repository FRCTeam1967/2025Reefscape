// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw254 extends SubsystemBase {
   private TalonFX claw;
   public double revsToMove;

   /** 
    * 
    */
   public Claw254() {
      //pivot instantiations
      claw = new TalonFX(Constants.Intake.CLAW_ID);
      var talonFXConfigs = new TalonFXConfiguration();

      //set slot configs
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Intake.kS; 
      slot0Configs.kV = Constants.Intake.kV;
      slot0Configs.kA = Constants.Intake.kA;
      slot0Configs.kP = Constants.Intake.kP;
      slot0Configs.kI = Constants.Intake.kI;
      slot0Configs.kD = Constants.Intake.kD;

      talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Intake.CURRENT_LIMIT));

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Intake.JERK;

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      claw.getConfigurator().apply(talonFXConfigs);

      //resetEncoders(); 
      claw.setNeutralMode(NeutralModeValue.Brake);
   }

   //pivot methods
   /** Stops pivot motor */
   public void stop() {
      claw.stopMotor();
   }

   /** Sets param to MotionMagic goal and gives request to pivot motor
    * @param revolutions - goal height
    */
   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.Intake.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      claw.setControl(request);
   }

   /** Resets encoders/ sets the position to 0 */
   public void resetEncoders() { 
      claw.setPosition(0);
   }
   
   public boolean isReached() {
      return Math.abs(((claw.getRotorPosition().getValueAsDouble()/Constants.Intake.GEAR_RATIO)*360) - ((revsToMove/Constants.Intake.GEAR_RATIO)*360)) < 5.0;
   }
   
   public void configDashboard(ShuffleboardTab tab) {
      tab.addNumber("ClawRelPosDeg",() -> ((claw.getRotorPosition()
      .getValueAsDouble()/Constants.Intake.GEAR_RATIO)*360))
      .withWidget(BuiltInWidgets.kTextView).withPosition(3, 0)
      .withSize(1, 1);
      tab.addBoolean("Claw Reached", () -> isReached())
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 0)
      .withSize(1, 1);
   }

   //intake methods
   public void periodic() {
   }
}