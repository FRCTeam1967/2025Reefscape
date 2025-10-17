package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;

public class Intake extends SubsystemBase {
   private TalonFX intakeMotor;

   public Intake() {
      intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
      
      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Intake.kS; 
      slot0Configs.kV = Constants.Intake.kV;
      slot0Configs.kA = Constants.Intake.kA;
      slot0Configs.kP = Constants.Intake.kP;
      slot0Configs.kI = Constants.Intake.kI;
      slot0Configs.kD = Constants.Intake.kD;

      intakeMotor.getConfigurator().apply(talonFXConfigs);

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      intakeMotor.getConfigurator().apply(talonFXConfigs);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */

   public void runIntake(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      intakeMotor.setControl(request);
   }

   public void stopMotor() {
      intakeMotor.stopMotor();
   }

   /**Adds value to shuffleboard
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addDouble("Top Intake Rel Pos",()->(intakeMotor.getRotorPosition().getValueAsDouble()))
      .withWidget(BuiltInWidgets.kTextView).withPosition(6, 0)
      .withSize(1, 1);

      tab.addBoolean("isReached",()->((intakeMotor.getRotorPosition().getValueAsDouble() >= Constants.Intake.INTAKE_ENCODER_STOP_VAL)))
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2)
      .withSize(1, 1);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
}