
\package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile; // Required for motion profiling
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Required for dashboard values to display
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Required as base class for subsystems
import frc.robot.Constants;
 // Required for constants defined in the project

// Classes for each branch level (L1, L2, L3, L4 branches)
// Classes can only be accessed by other classes in the same package, hence just "class"

public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor;
  private Canandcoder absEncoder;
  //private SparkPIDController pidController;
  private RelativeEncoder relativeEncoder;
  
  //private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(7,3);
  //private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  //public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  //public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  
  public double revsToMove;

  // in init function
  var talonFXConfigs = new TalonFXConfiguration();

  // set slot 0 gains
  var slot0Configs = talonFXConfigs.Slot0;
  slot0Configs.kS = Constants.Pivot.kS; // Add 0.25 V output to overcome static friction
  slot0Configs.kV = Constants.Pivot.kV; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kA = Constants.Pivot.kA; // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = Constants.Pivot.kP; // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = Constants.Pivot.kI; // no output for integrated error
  slot0Configs.kD = Constants.Pivot.kD; // no output for error derivative

  // set Motion Magic settings
  var motionMagicConfigs = talonFXConfigs.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.CRUISE_VELOCITY; // Target cruise velocity of 80 rps
  motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
  motionMagicConfigs.MotionMagicJerk = Constants.Pivot.JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)


  m_talonFX.getConfigurator().apply(talonFXConfigs);

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new TalonFX(Constants.Pivot.PIVOT_ID);
    pivotMotor.setInverted(true);    
    //pidController = pivotMotor.getPIDController();
    //pidController.setP(Constants.Pivot.kP);
    //pidController.setI(Constants.Pivot.kI);
    //pidController.setD(Constants.Pivot.kD);
    //pidController.setOutputRange(-0.2, 0.2);

    relativeEncoder = pivotMotor.getEncoder();
    
    absEncoder = new Canandcoder(Constants.Pivot.ENCODER_ID);
    pidController.setFeedbackDevice(relativeEncoder);

    Canandcoder.Settings settings = new Canandcoder.Settings();
    settings.setInvertDirection(true);
    absEncoder.setSettings(settings, 0.050);
  }

  /** Sets relative encoder value to absolute encoder value */
  public void setRelToAbs(){
    REVLibError success = relativeEncoder.setPosition(absEncoder.getAbsPosition()*Constants.Pivot.GEAR_RATIO);
    System.out.println("REV error" + success); 
  }

  /** Stops pivot motor */
  public void stop() {
    pivotMotor.stopMotor();
  }

  /**
   * @return position from absolute encoder
   */
  public double getAbsPos() {
    return absEncoder.getAbsPosition();
  }

  /**
   * Sets motion profiling goal to desired revolutions
   * @param revolutions
   */
  public void moveTo(double revolutions) {
    MotionMagicVoltage request = new MotionMagicVoltage(revsToMove).withFeedForward(Constants.Pivot.FEED_FORWARD);
    pivotMotor.setControl(request);
    //goal = new TrapezoidProfile.State(revolutions, 0);

    // set target position to desired revolutions
    m_talonFX.setControl(request.withPosition(revsToMove));
  }

  /**
   * @return whether profile has been finished
   */
  public boolean isReached(){
    return(profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /** Sets pivot motor to brake mode */
  public void setBrakeMode(){
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
    pidController.setReference(revs, CANSparkBase.ControlType.kPosition);

    SmartDashboard.putNumber("Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Abs Encoder", absEncoder.getAbsPosition());
    SmartDashboard.putNumber("Set Point", setpoint.position); 
    SmartDashboard.putNumber("revs", revs); 
    SmartDashboard.putNumber("Rel Pos Degrees", (relativeEncoder.getPosition()*360)/50);
    SmartDashboard.putNumber("Abs Encoder Degrees", absEncoder.getAbsPosition()*360);
  }
}
class L1 {
    private final Pivot pivot;

    public L1() {
        pivot = new Pivot(Constants.Pivot.L1_MOTOR1_ID, Constants.Pivot.L1_MOTOR2_ID);
    }

    public void moveTo(double position) {
        pivot.moveTo(position);
    }

    public void stop() {
        pivot.stop();
    }
    
    public double getAbsPos() {
        return pivot.getAbsPos();
    }
    
    public boolean isReached() {
        return pivot.isReached();
    }
}

class L2 {
    private final Pivot pivot;

    public L2() {
        pivot = new Pivot(Constants.Pivot.L2_MOTOR1_ID, Constants.Pivot.L2_MOTOR2_ID);
    }

    public void moveTo(double position) {
        pivot.moveTo(position);
    }

    public void stop() {
        pivot.stop();
    }
    
    public double getAbsPos() {
        return pivot.getAbsPos();
    }
    
    public boolean isReached() {
        return pivot.isReached();
    }
}

class L3 {
    private final Pivot pivot;

    public L3() {
        pivot = new Pivot(Constants.Pivot.L3_MOTOR1_ID, Constants.Pivot.L3_MOTOR2_ID);
    }

    public void moveTo(double position) {
        pivot.moveTo(position);
    }

    public void stop() {
        pivot.stop();
    }
    
    public double getAbsPos() {
        return pivot.getAbsPos();
    }
    
    public boolean isReached() {
        return pivot.isReached();
    }
}

class L4 {
    private final Pivot pivot;

    public L4() {
        pivot = new Pivot(Constants.Pivot.L4_MOTOR1_ID, Constants.Pivot.L4_MOTOR2_ID);
    }

    public void moveTo(double position) {
        pivot.moveTo(position);
    }

   public void stop() {
       pivot.stop();
   }
   
   public double getAbsPos() {
       return pivot.getAbsPos();
   }
   
   public boolean isReached() {
       return pivot.isReached();
   }
}