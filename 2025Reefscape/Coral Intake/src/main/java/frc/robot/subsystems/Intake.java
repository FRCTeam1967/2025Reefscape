package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    public Intake() {
        leftMotor = new TalonFX(Constants.Intake.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.Intake.RIGHT_MOTOR_ID);
        rightMotor.setInverted(true);
    }
    public void setMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
    public void stopMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}