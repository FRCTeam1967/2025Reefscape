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
    public static final int CLIMB_MOTOR_ID = 0;
    public static final double VELOCITY = 0;
    
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0; 
    public static final double kS = 0; 
    public static final double kV = 0; 
    public static final double kA = 0;

    public static final double CLIMB_ENCODER_STOP_VAL = 0;

    public static final double CRUISE_VELOCITY = 50.0;
    public static final double ACCELERATION = 70.0;
    public static final double JERK = 0.0;
   }

  public class Intake {
    public static final int TOP_INTAKE_MOTOR_ID = 0;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 1;

    public static final double INTAKE_SPEED = 0.5;
    public static final double EJECT_VELOCITY = -0.5;

    public static final double INTAKE_ENCODER_STOP_VAL = 0;
   }

  public class Pivot {
    public static final int PIVOT1_ID = 0;
    public static final int PIVOT2_ID = 0;
    
    public static final double GEAR_RATIO = 0/0;

    public static final double CRUISE_VELOCITY = 50.0;
    public static final double ACCELERATION = 70.0;
    public static final double JERK = 0.0;
    
    public static final double CONVERSION_FACTOR = 0.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double ERROR_THRESHOLD = 0.0;
    public static final double FEED_FORWARD = 0.0;


    public static final double CORAL_GROUND_INTAKE = 10 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // angle for coral ground intake
    public static final double CORAL_L1 = 55 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // angle for L1 scoring
    
    public static final double ALGAE_INTAKE = 0 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // angle for algae ground intake
    public static final double ALGAE_PROCESSOR = 0 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // angle for processor scoring

    //TODO: ask mech about hardstop
    public static final double PRE_CLIMB = 80 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // ~vertical, before climb
    public static final double CLIMB = 10 * Constants.Pivot.DEGREES_TO_REVOLUTIONS; // ~horizontal, finished climb position

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
    public static final double CURRENT_LIMIT = 40;
  }
   
}