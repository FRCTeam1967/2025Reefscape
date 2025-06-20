package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   private TalonFX intakeMotor;

   /** Initializes left motor, right motor, and beam break sensor IDs  */
   public Intake() {
      //kraken motors
      intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);

      var talonFXConfigs = new TalonFXConfiguration();
      intakeMotor.getConfigurator().apply(talonFXConfigs);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */

   public void runIntake(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      intakeMotor.setControl(request);
   }

   /** Stops both the left motor and right motor */
   public void stopMotor() {
      intakeMotor.stopMotor();
   }

   /**Adds value to shuffleboard
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addDouble("Intake Rel Pos",()->(intakeMotor.getRotorPosition().getValueAsDouble()))
      .withWidget(BuiltInWidgets.kTextView).withPosition(6, 0)
      .withSize(1, 1);

      tab.addBoolean("isReached",()->(intakeMotor.getRotorPosition().getValueAsDouble() >= Constants.Intake.INTAKE_ENCODER_STOP_VAL))
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2)
      .withSize(1, 1);

   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
}