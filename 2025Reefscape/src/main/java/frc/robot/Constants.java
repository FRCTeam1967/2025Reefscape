// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //paste here >>>
  public static class Xbox { 
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class Swerve {
    //motor/encoder ids
    public static final int FL_POWER = 7, FL_STEER = 8, FL_ENCODER = 4;
    public static final int FR_POWER = 1, FR_STEER = 2, FR_ENCODER = 1;
    public static final int BL_POWER = 5, BL_STEER = 6, BL_ENCODER = 3;
    public static final int BR_POWER = 3, BR_STEER = 4, BR_ENCODER = 2;

    public static final double FL_OFFSET = -132.275390625/360; //-134.560546875/360;
    public static final double FR_OFFSET = 2.197265625/360;//107.40234375/360;
    public static final double BL_OFFSET = 116.630859375/360; //116.630859375/360;
    public static final double BR_OFFSET = -18.544921875/360; //-17.40234375/360;
    public static final int PIGEON_GYRO = 9;

    // public static final double FL_OFFSET = -132.01171875/360; //-134.560546875/360;
    // public static final double FR_OFFSET = 4.39453125/360;//107.40234375/360;
    // public static final double BL_OFFSET = 117.421875/360; //116.630859375/360;
    // public static final double BR_OFFSET = -17.2265625/360; //-17.40234375/360;

    //janky offsets
    // public static final double FL_OFFSET = 171.507813/360;
    // public static final double FR_OFFSET = -59.050391/360;
    // public static final double BL_OFFSET = 119.619141/360;
    // //public static final double BR_OFFSET = (-150.820313-180)/360;
    // public static final double BR_OFFSET = -150.820313/360;

    // public static final double FL_OFFSET = -10.0195/360;
    // public static final double FR_OFFSET = 117.6855/360;
    // public static final double BL_OFFSET = -52.7343/360;
    // public static final double BR_OFFSET = 27.1582/360;


    //pid values
    public static final double POWER_kS = 0.14;//0.14; 
    public static final double POWER_kV = 12/(100/8.14); //1.25; 
    public static final double POWER_kA = 0; 
    public static final double POWER_kP = POWER_kV * 0.8; //0.25;
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kS = 0; //0.1
    public static final double STEER_kV = 0; //30
    public static final double STEER_kA = 0; // 15
    public static final double STEER_kP = 12; //100
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0.5;

    //gear ratios + meter conversions
    public static final double STEER_GEAR_RATIO = 150.0/7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MK4I_L1_REV_TO_METERS = WHEEL_CIRCUMFERENCE;
    public static final double RPM_TO_MPS = MK4I_L1_REV_TO_METERS / 60.0;
    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = STEER_GEAR_RATIO;
    public static final double REVERSE_OFFSET = Units.inchesToMeters(2.0) * Math.PI;
    public static final double METERS_TO_ENC_COUNT = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    

    //distances/measurements
    public static final double SWERVE_MAX_SPEED = 4.1695; //m/s
    public static final double WIDTH = Units.inchesToMeters(23), LENGTH = Units.inchesToMeters(23);
    public static final double SWERVE_AMP_OFFSET = 0.3083496; //rotations of encoder
    public static final double AMP_REVERSE_JS_INPUT = 0.4; //joystick input

    //max speeds
    public static final double ROTATION_CIRCLE_CIRCUMFERENCE = (WIDTH / Math.sqrt(2.0)) * 2.0 * Math.PI;
    public static final double SWERVE_ROTATION_MAX_SPEED_IN_RAD = (SWERVE_MAX_SPEED / ROTATION_CIRCLE_CIRCUMFERENCE) * 2.0 * Math.PI; 

    //kinematics
    public static Translation2d m_frontLeftLocation = new Translation2d(LENGTH / 2, WIDTH / 2);
    public static Translation2d m_frontRightLocation = new Translation2d(LENGTH / 2, -WIDTH / 2);
    public static Translation2d m_backLeftLocation = new Translation2d(-LENGTH / 2, WIDTH / 2);
    public static Translation2d m_backRightLocation = new Translation2d(-LENGTH / 2, -WIDTH / 2);

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = 
      new SwerveDriveKinematics(
        m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation, 
        m_backRightLocation
      );
    
    public static final double SWERVE_ROTATION_TOLERANCE = 5;

    public static final TrapezoidProfile.Constraints SWERVE_ROTATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(36000, 36000);
    public static final TrapezoidProfile.Constraints SWERVE_TRANSLATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(15, 3);
  }

  public static class Vision {
    public static final double DEGREE_ERROR = 4.0;

    public static final double LIMELIGHT_ANGLE_DEGREES = 0; //need to verify
    public static final double LIMELIGHT_HEIGHT_INCHES = 25.125; //need to verify
    public static final double TARGET_HEIGHT_INCHES = 54; //need to verify
  }

  public static class Auto {
    public static final double PIVOT_INTAKE_TIMEOUT = 1.0, PIVOT_UP_TIMEOUT = 2.0, SHOOT_SPEAKER_TIMEOUT = 3.0; //pivot timeout: 2
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
    public static final double kMaxAngularSpeedDegreesPerSecond = 180;
    public static final double kMaxAngularSpeedDegreesPerSecondSquared = 180;

    public static final double APRIL_TAG_HEIGHT_INCHES = 48.125; //need to verify
    public static final double LIMELIGHT_ANGLE_DEGREES = 45.0; //need to measure
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 37.0; //need to measure

    public static final double kPXController = 25, kPYController = 30, kPThetaController = 6.5; //7.5

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedDegreesPerSecond, kMaxAngularSpeedDegreesPerSecondSquared);
    public static final double RADIANS_TO_DEGREES = 180.0/Math.PI;
  } 

  public static class Elevator {//need to check all
    public static final int LEFT_MOTOR_IDX = 11;
    public static final int RIGHT_MOTOR_IDX = 10;

    public static final double CRUISE_VELOCITY = 150; //200 //150 //100
    public static final double ACCELERATION = 200; //300 //250 //150 //200 //100
    public static final double JERK = 1300; //1000 //950 //750 //600 //1000 //1600 <---how fast the acceleration is reached
    public static final double FEED_FORWARD = 0;
    public static final double SAFE_SPEED_FACTOR = 0.4; //based on climb testing
    public static final double SAFE_SPEED = -0.4; //0.8 was for climb, need to test

    public static final int SENSOR_ID = 1;

    public static final double GEAR_RATIO = 64/8;
    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.76*Math.PI;
    
    public static final double UP = 26.5; //25 //26.5 //DO NOT CHANGE---MAX PHYSICAL HEIGHT
    public static final double MIDDLE = 12;
    public static final double SAFE = 1;

    //public static final double EXTEND_ROTATIONS = INCHES*(GEAR_RATIO/SPROCKET_PITCH_CIRCUMFERENCE);


    //public static final double SPEED = 25;
    public static final double kP = 2.5; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0; // A velocity error of 1 rps results in 0.1 V output
    public static final double kG = 0; 
    public static final double kS = 0; 
    public static final double kV = 0; 
    public static final double kA = 0 ;



    // public static final double GEAR_RATIO = 5; 
    // public static final double FEED_FORWARD = 0.07;
    // public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.751 * Math.PI; 

    // public static final double ELEVATOR_MAX_HEIGHT = 0.07; 
    // public static final int SLOT_IDX_VALUE = 0; 

    // public static final double CRUISE_VELOCITY = 80; // Target cruise velocity of 80 rps
    // public static final double ACCELERATION = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    // public static final double JERK = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // //public static final int FALCON_ENCODER_TICKS_PER_REVOLUTION = 2048;

   public static final double ERROR_THRESHOLD = 2;//0.5; 
    // public static final double EXTEND_HEIGHT = 1000; //need to check
  }

  public class AlgaeMechanism {
    public static final int PIVOT_ID = 14;
    public static final int INTAKE_ID = 15;
    //public static final int ENCODER_ID = 15;

    public static final double ALGAE_INTAKE_SPEED = 10.0;
    
    //public static final double GEAR_RATIO = 16/1.0;
    //public static final double SPROCKET_PITCH_CIRCUMFERENCE = 3.19;
    public static final double GEAR_RATIO = 28.5; //27/1.0
    public static final double CRUISE_VELOCITY = 100.0;
    public static final double ACCELERATION = 200.0;
    public static final double JERK = 1600.0;

    public static final double CONVERSION_FACTOR = 0.002777777777777778;
    //public static final double INTAKE_SAFE = 0.31666666666666665;
    //public static final double INTAKE_DOWN = 0.02777777777777778;

    public static final double kP = 4.8;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    public static final double ERROR_THRESHOLD = 5.0; //in degrees not revolutions
    public static final double FEED_FORWARD = 0.0;

    public static final double UP = 194.6 * Constants.AlgaeMechanism.DEGREES_TO_REVOLUTIONS;
    public static final double MIDDLE = 158.9* Constants.AlgaeMechanism.DEGREES_TO_REVOLUTIONS;
    public static final double SAFE = 0.0 * Constants.AlgaeMechanism.DEGREES_TO_REVOLUTIONS;

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
  }

  public static class Extender{
    public static final int MOTOR_IDX = 13;
    public static final int SENSOR_ID = 8000;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double GEAR_RATIO = 5; //Need to check
    public static final double FEED_FORWARD = 0.07;  //Need to check
    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 0; //Need to check

    public static final double ELEVATOR_MAX_HEIGHT = 0.07; //Need to check
    public static final int SLOT_IDX_VALUE = 0; //Need to check

    public static final double CRUISE_VELOCITY = 0; //Need to check value
    public static final double ACCELERATION = 0; //Need to check value
    public static final double JERK = 0; //Need to check

    public static final int CANSPARKMAX_ENCODER_TICKS_PER_REVOLUTION = 42;

    public static final double ERROR_THRESHOLD = 0.5; //need to check

    public static final double EXTEND_DISTANCE = 1000; //need to check
    public static final double SAFE = 100000000;

  }
  public static final double ROBOT_PERIOD = 0;
}
