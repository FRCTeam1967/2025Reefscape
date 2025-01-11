// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   private final TalonFX leftMotor = new TalonFX(20);
   private final TalonFX rightMotor = new TalonFX(21);

   public Intake() {
   }

   public void setMotors(double speed) {
      this.leftMotor.set(speed);
      this.rightMotor.set(speed * -1.0);
   }

   public void stopMotors() {
      this.leftMotor.stopMotor();
      this.rightMotor.stopMotor();
   }
}
