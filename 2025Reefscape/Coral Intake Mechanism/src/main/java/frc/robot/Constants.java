package frc.robot;

public final class Constants {
  public static class Pivot {
    // Motor and Sensor IDs
    public static final int PIVOT_ID = 14;

    // absolute encoder
    public static final int ENCODER_ID = 15;
    
    // Mechanical Constants
    public static final double GEAR_RATIO = 50.0 / 1.0;
    
    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 100; // Units: rotations per second
    public static final double ACCELERATION = 200; // Units: rotations per second^2
    public static final double JERK = 1600; // Units: rotations per second^3
    
    // Position Constants
    public static final double CONVERSION_FACTOR = 1.0 / 360.0; // Converts degrees to rotations
    public static final double INTAKE_SAFE = 114 * CONVERSION_FACTOR;
    public static final double INTAKE_DOWN = 10 * CONVERSION_FACTOR;

    // PID and Feedforward Constants
    public static final double kP = 4.8;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double kS = 0.25; // Static friction compensation
    public static final double kV = 0.12; // Velocity feedforward
    public static final double kA = 0.01; // Acceleration feedforward
    
    // Motion Control Constants
    public static final double ERROR_THRESHOLD = 5; // In rotations
    public static final double FEED_FORWARD = 0.0; // Adjust if needed
 
}
  public static class Intake {
    public static final int LEFT_MOTOR_ID = 20; //adjust as needed
    public static final int RIGHT_MOTOR_ID = 21; //adjust as needed
  }
}