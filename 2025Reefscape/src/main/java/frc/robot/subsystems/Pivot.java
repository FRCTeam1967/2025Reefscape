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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
   private TalonFX pivotMotor = new TalonFX(14);
   private CANcoder absEncoder;
   public double revsToMove;

   public Pivot() {
      TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
      this.absEncoder = new CANcoder(15);
      Slot0Configs slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = 0.25;
      slot0Configs.kV = 0.12;
      slot0Configs.kA = 0.01;
      slot0Configs.kP = 4.8;
      slot0Configs.kI = 0.0;
      slot0Configs.kD = 0.1;
      MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 100.0;
      motionMagicConfigs.MotionMagicAcceleration = 200.0;
      motionMagicConfigs.MotionMagicJerk = 1600.0;
      this.pivotMotor.getConfigurator().apply(talonFXConfigs);
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      this.pivotMotor.getConfigurator().apply(config);
      this.pivotMotor.setNeutralMode(NeutralModeValue.Brake);
   }

   public void stop() {
      this.pivotMotor.stopMotor();
   }

   public double getPosition() {
      return this.pivotMotor.getRotorPosition().getValueAsDouble();
   }

   public void moveTo(double revolutions) {
      this.revsToMove = revolutions;
      MotionMagicVoltage request = (new MotionMagicVoltage(this.revsToMove)).withFeedForward(0.0);
      this.pivotMotor.setControl(request);
   }

   public void setRelToAbs() {
      this.pivotMotor.setPosition(this.absEncoder.getAbsolutePosition().getValueAsDouble() * 50.0);
   }

   public boolean isReached() {
      return Math.abs(this.getPosition() - this.revsToMove) < 5.0;
   }

   public void periodic() {
      SmartDashboard.putNumber("Pivot Abs Position", this.absEncoder.getAbsolutePosition().getValueAsDouble() * 50.0);
      SmartDashboard.putNumber("Pivot Rel Position", this.getPosition());
      SmartDashboard.putNumber("Pivot Target", this.revsToMove);
      SmartDashboard.putBoolean("Pivot At Target", this.isReached());
   }
}