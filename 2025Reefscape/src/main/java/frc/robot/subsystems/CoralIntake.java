package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

//beambreak sensor = digital input
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
   private DigitalInput sensor;
   private TalonFX intakeMotor;

   /** Initializes left motor, right motor, and beam break sensor IDs */
   public CoralIntake() {
      //kraken motors
      intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
      sensor = new DigitalInput(Constants.Intake.BEAM_ID);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction */
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

   public void periodic() {
      
      SmartDashboard.putBoolean("Beambreak Sensor", isBroken());

   }

   public void configDashboard(ShuffleboardTab tab) {
      tab.add("Beam Break Sensor Detected?", !(sensor.get()));
   }


}