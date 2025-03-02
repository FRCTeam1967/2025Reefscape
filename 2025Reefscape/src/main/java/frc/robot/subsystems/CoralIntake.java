package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
   private DigitalInput sensor;
   private TalonFX intakeMotor;

   /** Initializes left motor, right motor, and beam break sensor IDs  */
   public CoralIntake() {
      //kraken motors
      intakeMotor = new TalonFX(Constants.CoralIntake.INTAKE_MOTOR_ID);
      sensor = new DigitalInput(Constants.CoralIntake.BEAM_ID);

      var talonFXConfigs = new TalonFXConfiguration();


      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.CoralIntake.kS; 
      slot0Configs.kV = Constants.CoralIntake.kV; 
      slot0Configs.kA = Constants.CoralIntake.kA; 
      slot0Configs.kP = Constants.CoralIntake.kP; 
      slot0Configs.kI = Constants.CoralIntake.kI;
      slot0Configs.kD = Constants.CoralIntake.kD; 
  
      // set Motion Magic settings
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.CoralIntake.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.CoralIntake.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.CoralIntake.JERK;

      talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

  

            
      intakeMotor.setNeutralMode(NeutralModeValue.Brake);
      //talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      intakeMotor.getConfigurator().apply(talonFXConfigs);

      intakeMotor.getPosition().setUpdateFrequency(150);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */
   public void setMotor(double speed) {
      intakeMotor.set(speed);
   }

   public void moveTo(double revolutions) {
      MotionMagicVoltage request = (new MotionMagicVoltage(revolutions)).withFeedForward(0.0);
      intakeMotor.setControl(request);
   }

   public void setVelocity(double velocity) {
      //MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(velocity);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity);

      intakeMotor.setControl(request);

   }
   public void moveMotor(double speed) {
      VelocityVoltage request = new VelocityVoltage(speed);
      intakeMotor.setControl(request);
   }

   /** Stops both the left motor and right motor */
   public void stopMotor() {
      intakeMotor.stopMotor();
   }

   /** Checks if the beam in the beam break sensor has been broken */
   public boolean isBroken(){
      return !(sensor.get());
   }

   public double getRotorPos() {
      return intakeMotor.getRotorPosition().getValueAsDouble();
   }

   public void zeroEncoder() {
     intakeMotor.setPosition(0);
   }



   public void periodic() {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
      SmartDashboard.putBoolean("Beambreak Sensor", isBroken());

   }
   /**Adds value to shuffleboard
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addBoolean("CBeamBreak?", ()-> !(sensor.get()))
      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 0)
      .withSize(1, 1);
      tab.addDouble("Intake Rel Pos",()->(intakeMotor.getRotorPosition().getValueAsDouble()));
      tab.addBoolean("isReached",()->(intakeMotor.getRotorPosition().getValueAsDouble() >= Constants.CoralIntake.INTAKE_ENCODER_STOP_VAL));

   }


}