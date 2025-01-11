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
  public class Pivot {
    public static final int PIVOT_ID = 14;
    public static final int ENCODER_ID = 15;
    
    public static final double GEAR_RATIO = 50.0;
    public static final double CRUISE_VELOCITY = 100.0;
    public static final double ACCELERATION = 200.0;
    public static final double JERK = 1600.0;

    public static final double CONVERSION_FACTOR = 0.002777777777777778;
    public static final double INTAKE_SAFE = 0.31666666666666665;
    public static final double INTAKE_DOWN = 0.02777777777777778;

    public static final double kP = 4.8;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    public static final double ERROR_THRESHOLD = 5.0;
    public static final double FEED_FORWARD = 0.0;

    public static final double UP = 100.0;
    public static final double MIDDLE = 50.0;
    public static final double SAFE = 0.0;
  }

  public class Intake {
    public static final int LEFT_MOTOR_ID = 20;
    public static final int RIGHT_MOTOR_ID = 21;

    public static final double SLOW = 0.25;
    public static final double HIGH = 0.5;

    public static final int BEAM_ID = 8;
   }
    
 
}
