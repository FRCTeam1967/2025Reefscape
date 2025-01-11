package frc.robot;

public final class Constants {
  public static class Pivot {
    public static final int PIVOT_ID = 14, ENCODER_ID = 32;
    public static final double GEAR_RATIO = 50/1;

    public static final double CRUISE_VELOCITY = 100;
    public static final double ACCELERATION = 200;
    public static final double JERK = 1600;
    public static final double FEED_FORWARD = 0;
    
    public static final double CONVERSION_FACTOR = 1.0/360.0;
    public static final double INTAKE_SAFE = 114 * Constants.Pivot.CONVERSION_FACTOR;
    public static final double INTAKE_DOWN = 10 * Constants.Pivot.CONVERSION_FACTOR;

    public static final double kP = 4.8; 
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    //public static final double kG = 0;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

  }
}