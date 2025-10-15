package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class Pivot extends SubsystemBase {
   //private TalonFX pivotMotor1;
   private TalonFX pivotMotor2;
   public double revsToMove;
   private TalonFXConfiguration config;
   
   public Pivot() {
      //pivotMotor1 = new TalonFX(Constants.Pivot.PIVOT1_ID);
      pivotMotor2 = new TalonFX(Constants.Pivot.PIVOT2_ID);
      
      config = new TalonFXConfiguration();

      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = Constants.Pivot.kS; 
      slot0Configs.kV = Constants.Pivot.kV;
      slot0Configs.kA = Constants.Pivot.kA;
      slot0Configs.kP = Constants.Pivot.kP;
      slot0Configs.kI = Constants.Pivot.kI;
      slot0Configs.kD = Constants.Pivot.kD;

      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY;
      motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION;
      motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK;

      // inverted value?
      talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      
      //pivotMotor1.getConfigurator().apply(talonFXConfigs);
      pivotMotor2.getConfigurator().apply(talonFXConfigs);

      //pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
      pivotMotor2.setNeutralMode(NeutralModeValue.Brake);

      config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Pivot.CURRENT_LIMIT));
   }

   public void stop() {
      //pivotMotor1.stopMotor();
      pivotMotor2.stopMotor();
   }

   /** creates a Motion Magic request, sets target position to revsToMove 
    * @param - revolutions
   */
   public void moveTo(double revolutions) {
      revsToMove = revolutions*(Constants.Pivot.GEAR_RATIO); 
      MotionMagicVoltage request = (new MotionMagicVoltage(revsToMove)).withFeedForward(0.0);
      //pivotMotor1.setControl(request);
      pivotMotor2.setControl(request);
   }
   
   public void resetEncoders() { 
      //pivotMotor1.setPosition(0);
      pivotMotor2.setPosition(0);
   }

   public boolean isReached() {
      return Math.abs(((pivotMotor2.getRotorPosition().getValueAsDouble()/Constants.Pivot.GEAR_RATIO)*360) - ((revsToMove/Constants.Pivot.GEAR_RATIO)*360)) < 5.0;
   }

   public void maintainPosition() {
      moveTo(Constants.Pivot.CORAL_L1);
   }

   /**Adds values to shuffleboard 
    * @param - tab
    */
   public void configDashboard(ShuffleboardTab tab) {
      /*tab.addNumber("PivotRelPosDeg",() -> ((pivotMotor1.getRotorPosition()
      .getValueAsDouble())/Constants.Pivot.GEAR_RATIO)*360)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(7, 0).withSize(1, 1);
      tab.addBoolean("Pivot At Target", () -> isReached()).withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(0, 1).withSize(1, 1);*/
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
}