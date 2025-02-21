package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CoralPivot extends SubsystemBase {
   private TalonFX pivotMotor;
   private DigitalInput limitSwitch;
   //private CANcoder absEncoder; 
   public double revsToMove;
   private TalonFXConfiguration config;
   
   //* Creates a new pivot */
   public CoralPivot() {
      //absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID); 
      pivotMotor = new TalonFX(Constants.CoralPivot.PIVOT_ID);
      limitSwitch = new DigitalInput(Constants.CoralPivot.SWITCH_ID);

      config = new TalonFXConfiguration();

      var talonFXConfigs = new TalonFXConfiguration();

      //* sets  slot configs */
      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.CoralPivot.kS; 
      slot0Configs.kV = Constants.CoralPivot.kV;
      slot0Configs.kA = Constants.CoralPivot.kA;
      slot0Configs.kP = Constants.CoralPivot.kP;
      slot0Configs.kI = Constants.CoralPivot.kI;
      slot0Configs.kD = Constants.CoralPivot.kD;


      //sets motion magic configurations
      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.CoralPivot.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.CoralPivot.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.CoralPivot.JERK;

      talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      //applies motion magic configs
      pivotMotor.getConfigurator().apply(talonFXConfigs);

      //resetEncoders(); 
      pivotMotor.setNeutralMode(NeutralModeValue.Brake);
      checkLimit();

      config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.CoralPivot.CURRENT_LIMIT));
   }

   /** Stops the pivot motor */
   public void stop() {
      pivotMotor.stopMotor();
   }

   /** creates a Motion Magic request, sets target position to revsToMove 
    * @param - revolutions
   */
   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.CoralPivot.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      pivotMotor.setControl(request);
   }
   
   /** checks if pivot has surpassed hard stop
    * if limit has been reached, sets pivot motor speed to 0 */
   public void checkLimit(){
      if (!limitSwitch.get()){
         pivotMotor.setPosition(0);
      }
   }

   /** zeroes the motor (sets its position to 0) */
   public void resetEncoders() { 
      pivotMotor.setPosition(0);
   }

   /** checks if the position the motor is at is within error threshold of the end goal */
   public boolean isReached() {
      return Math.abs(((pivotMotor.getRotorPosition().getValueAsDouble()/Constants.CoralPivot.GEAR_RATIO)*360) - (revsToMove*360)) < 5.0;
   }

   public void periodic() {
      SmartDashboard.putNumber("Pivot Rel Position Degrees",(pivotMotor.getRotorPosition().getValueAsDouble()/Constants.CoralPivot.GEAR_RATIO)*360);
      SmartDashboard.putBoolean("Pivot At Target", isReached());
      SmartDashboard.putBoolean("Coral Pivot Sensor", !limitSwitch.get());
      checkLimit();
   }

   /**Adds values to shuffleboard 
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      tab.addNumber("CPivotRelPosDeg",() -> (pivotMotor.getRotorPosition()
      .getValueAsDouble()/Constants.CoralPivot.GEAR_RATIO)*360).withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1).withSize(1, 1);
      tab.addBoolean("CPivot At Target", () -> isReached()).withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(1, 1).withSize(1, 1);
      tab.addBoolean("CPivot Sensor?", () -> !limitSwitch.get()).withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(2, 1).withSize(1, 1);
   }
}