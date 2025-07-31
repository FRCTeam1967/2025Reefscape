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

  //paste here >>>
  public static class Xbox { 
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }

  public class Intake {
    //CLAW / OPENING PART CONSTANTS
    public static final int CLAW_ID = 40;

    //public static final double GEAR_RATIO = 16/1.0;
    //public static final double SPROCKET_PITCH_CIRCUMFERENCE = 3.19;
    public static final double GEAR_RATIO = 28.5/1.0;
    public static final double CRUISE_VELOCITY = 400.0;
    public static final double ACCELERATION = 600.0;
    public static final double JERK = 1600.0;

    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    public static final double INTAKE_kS = 0.0;
    public static final double INTAKE_kV = 0.0;
    public static final double INTAKE_kA = 0.0;
    public static final double INTAKE_kP = 0.5;
    public static final double INTAKE_kI = 0.0;
    public static final double INTAKE_kD = 0.0;

    public static final double INTAKE_CRUISE_VELOCITY = 50;
    public static final double INTAKE_ACCELERATION = 100;
    public static final double INTAKE_JERK = 150;

    public static final double ERROR_THRESHOLD = 5.0; //in degrees not revolutions
    public static final double FEED_FORWARD = 0.0;

    public static final double CLAW_OPEN = -2 * Constants.Intake.DEGREES_TO_REVOLUTIONS;
    public static final double CLAW_CLOSE = 10 * Constants.Intake.DEGREES_TO_REVOLUTIONS;

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;

    //INTAKE ROLLER CONSTANTS
    public static final int SIDE_ROLLER_ID = 41;
    public static final int TOP_ROLLER_ID = 42;

    public static final double SIDE_INTAKE_SPEED = 0.5;
    public static final double SIDE_OUTTAKE_SPEED = -0.5;
    public static final double SIDE_STOP_SPINNING = 0.0;

    public static final double TOP_INTAKE_SPEED = -0.1;//-0.4;
    public static final double TOP_OUTTAKE_SPEED = 0.1;//0.4;
    public static final double TOP_STOP_SPINNING = 0.0;

    public static final double CURRENT_LIMIT = 35;
  }
}
