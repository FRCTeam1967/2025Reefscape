package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
   private TalonFX climbMotor;

   public Climb() {
      climbMotor = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);

      var talonFXConfigs = new TalonFXConfiguration();

      climbMotor.setNeutralMode(NeutralModeValue.Brake);

      climbMotor.getConfigurator().apply(talonFXConfigs);
      climbMotor.getPosition().setUpdateFrequency(150);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */

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