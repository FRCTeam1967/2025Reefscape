// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

//beambreak sensor = digital input
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   private DigitalInput sensor;
   private TalonFX leftMotor;
   private TalonFX rightMotor;

   /** Initializes left motor, right motor, and beam break sensor IDs */
   public Intake() {
      //kraken motors
      leftMotor = new TalonFX(20);
      rightMotor = new TalonFX(21);
      sensor = new DigitalInput(Constants.Intake.BEAM_ID);
   }

   /**  Sets speed for right and left motors, left motor is reversed for intake to run in opposite direction */
   public void setMotors(double speed) {
      leftMotor.set(speed);
      rightMotor.set(speed * -1.0);
   }

   /** Stops both the left motor and right motor */
   public void stopMotors() {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
   }

   /** Checks if the beam in the beam break sensor has been broken */
   public boolean isBroken(){
      return !(sensor.get());
   }

}
