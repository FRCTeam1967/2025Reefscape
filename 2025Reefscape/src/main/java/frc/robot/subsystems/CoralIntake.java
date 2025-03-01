package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction
    * @param - speed
    */
   public void setMotor(double speed) {
      intakeMotor.set(speed);
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