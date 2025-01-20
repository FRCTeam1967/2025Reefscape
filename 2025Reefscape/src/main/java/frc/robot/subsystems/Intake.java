// Source code is decompiled from a .class file using FernFlower decompiler.
/* 
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   private DigitalInput sensor;
   private TalonFX leftMotor;
   private TalonFX rightMotor;

   //private final TalonFX leftMotor = new TalonFX(20);
   //private final TalonFX rightMotor = new TalonFX(21);
   //private final DigitalInput sensor = new DigitalInput(Constants.Intake.BEAM_ID);

   public Intake() {
      leftMotor = new TalonFX(20);
      rightMotor = new TalonFX(21);
      sensor = new DigitalInput(Constants.Intake.BEAM_ID);
   }

   public void setMotors(double speed) {
      leftMotor.set(speed);
      rightMotor.set(speed * -1.0);
   }

   public void stopMotors() {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
   }

   public boolean isBroken(){
      return !(sensor.get());
   }

}

*/
