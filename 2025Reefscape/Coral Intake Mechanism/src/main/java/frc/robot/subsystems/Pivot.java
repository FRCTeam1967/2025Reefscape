package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    private TalonFX pivotMotor; //using only 1 pivot motor
    private CANcoder absEncoder;
    public double revsToMove;

    public Pivot() {
        pivotMotor = new TalonFX(Constants.Pivot.PIVOT_ID); //defines the pivot ID
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID); //defines the encoder ID
        

        // Set slot 0 gains / all are PID values
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Pivot.kS; 
        slot0Configs.kV = Constants.Pivot.kV;
        slot0Configs.kA = Constants.Pivot.kA;
        slot0Configs.kP = Constants.Pivot.kP;
        slot0Configs.kI = Constants.Pivot.kI;
        slot0Configs.kD = Constants.Pivot.kD;

        // Set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK;

        // Apply configurations
        pivotMotor.getConfigurator().apply(talonFXConfigs);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor.getConfigurator().apply(config);

        // Set neutral mode to brake
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void stop() {
        pivotMotor.stopMotor();
    }

    public double getPosition() {
        return pivotMotor.getRotorPosition().getValueAsDouble(); //gets the position as a double value
    }

    public void moveTo(double revolutions) {
        revsToMove = revolutions; //setting the revsToMove equal to revolutions
        MotionMagicVoltage request = new MotionMagicVoltage(revsToMove).withFeedForward(Constants.Pivot.FEED_FORWARD);
        pivotMotor.setControl(request);
    }

    public void setRelToAbs(){
        pivotMotor.setPosition((absEncoder.getAbsolutePosition().getValueAsDouble())*Constants.Pivot.GEAR_RATIO);
    }
    
    public boolean isReached() {
        return Math.abs(getPosition() - revsToMove) < Constants.Pivot.ERROR_THRESHOLD;
    }

    @Override
    public void periodic() { //prints all the below values on SmartDashboard
        SmartDashboard.putNumber("Pivot Abs Position", (absEncoder.getAbsolutePosition().getValueAsDouble())*Constants.Pivot.GEAR_RATIO);
        SmartDashboard.putNumber("Pivot Rel Position", getPosition());
        SmartDashboard.putNumber("Pivot Target", revsToMove);
        SmartDashboard.putBoolean("Pivot At Target", isReached());
    }
}
