// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Xbox { 
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }

  public class Climb {
    public static final int CLIMB_MOTOR_ID = 13;
    public static final double VELOCITY = 15.0;
    public static final double CLOSING_VELOCITY = -15.0;

    public static final double GEAR_RATIO = 75/1;

    public static final double CRUISE_VELOCITY = 50.0;
    public static final double ACCELERATION = 70.0;
    public static final double JERK = 0.0;

    public static final double CONVERSION_FACTOR = 0.0;
    
    public static final double kP = 25; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0; 
    public static final double kV = 0; 
    public static final double kA = 0;

    public static final double CLIMB_ENCODER_STOP_VAL = 0;

    public static final double ERROR_THRESHOLD = 0.0;
    public static final double FEED_FORWARD = 0.0;

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
    public static final double CURRENT_LIMIT = 40;
   }

  public class Intake {
    public static final int INTAKE_MOTOR_ID = 10;

    public static final double kP = 0.1; //10
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0; 
    public static final double kV = 0; 
    public static final double kA = 0;

    public static final double IDLE_SPEED = 15.0; //15.0
    public static final double INTAKE_SPEED = 100.0; //15.0
    public static final double ALGAE_INTAKE_SPEED = -450.0; //150.0
    public static final double EJECT_VELOCITY = -15.0;
    public static final double DESCORE_VELOCITY = -55.0;

    public static final double INTAKE_ENCODER_STOP_VAL = 0;
    public static final double CURRENT_LIMIT = 30;
   }

  public class Pivot {
    public static final int PIVOT1_ID = 11;
    public static final int PIVOT2_ID = 12;
    
    public static final double GEAR_RATIO = 144/1;

    public static final double CRUISE_VELOCITY = 50.0;
    public static final double ACCELERATION = 70.0;
    public static final double JERK = 0.0;
    
    public static final double CONVERSION_FACTOR = 0.0;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double ERROR_THRESHOLD = 0.0;
    public static final double FEED_FORWARD = 0.0;


    public static final double CORAL_GROUND_INTAKE = -90.5 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -93.5
    public static final double CORAL_L1 = -42.3 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -45.3
    
    public static final double ALGAE_INTAKE = -51.7 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -60.7
    public static final double ALGAE_PROCESSOR = -51.7 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -60.7
    public static final double ALGAE_DESCORE = -30.8 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -43.8

    public static final double PRE_CLIMB = 6.1 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // 9.1
    public static final double CLIMB = -80.5 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // -45.3

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
    public static final double CURRENT_LIMIT = 40;
  }
   
}