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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake {
    public static final int MOTOR_ID = 7;
  }

  public static class Shooter {
    public static final int TOP_MOTOR_ID = 4;
    public static final int BOTTOM_MOTOR_ID = 5;
  }

  public static class Elevator {
    public static final int MOTOR_ID_ONE = 17;
    public static final int MOTOR_ID_TWO = 1;

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0; 
    public static final double kS = 0.0; 
    public static final double kV = 0.0; 
    public static final double kA = 0.0;

    public static final double CRUISE_VELOCITY = 160.0; //4.0
    public static final double ACCELERATION = 240.0; //30.0
    public static final double JERK = 350.0; //0.1
  }

}
