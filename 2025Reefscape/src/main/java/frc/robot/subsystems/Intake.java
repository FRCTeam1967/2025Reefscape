package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   private TalonFX topMotor;
   private TalonFX bottomMotor;

   public Intake() {
      topMotor = new TalonFX(Constants.Intake.TOP_INTAKE_MOTOR_ID);
      bottomMotor = new TalonFX(Constants.Intake.BOTTOM_INTAKE_MOTOR_ID);

      var talonFXConfigs = new TalonFXConfiguration();
      topMotor.getConfigurator().apply(talonFXConfigs);
      bottomMotor.getConfigurator().apply(talonFXConfigs);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */

   public void runIntake(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      topMotor.setControl(request);
      bottomMotor.setControl(request);
   }

   public void stopMotor() {
      topMotor.stopMotor();
      bottomMotor.stopMotor();
   }

   /**Adds value to shuffleboard
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addDouble("Top Intake Rel Pos",()->(topMotor.getRotorPosition().getValueAsDouble()))
      .withWidget(BuiltInWidgets.kTextView).withPosition(6, 0)
      .withSize(1, 1);

      tab.addDouble("Bottom Intake Rel Pos",()->(bottomMotor.getRotorPosition().getValueAsDouble()))
      .withWidget(BuiltInWidgets.kTextView).withPosition(7, 0)
      .withSize(1, 1);

      tab.addBoolean("isReached",()->((topMotor.getRotorPosition().getValueAsDouble() >= Constants.Intake.INTAKE_ENCODER_STOP_VAL) 
                                       && (bottomMotor.getRotorPosition().getValueAsDouble() >= Constants.Intake.INTAKE_ENCODER_STOP_VAL)))
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2)
      .withSize(1, 1);
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
}