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
  public static class Xbox {
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }
  

  public static class Elevator {//need to check all
    public static final int LEFT_MOTOR_IDX = 11;
    public static final int RIGHT_MOTOR_IDX = 10;

    public static final double CRUISE_VELOCITY = 100;
    public static final double ACCELERATION = 150; //200 //100
    public static final double JERK = 600; //1000 //1600 <---how fast the acceleration is reached
    public static final double FEED_FORWARD = 0;

    public static final int SENSOR_ID = 1;

    public static final double GEAR_RATIO = 64/8;
    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.76*Math.PI;
    
    public static final double UP = 26.5; //DO NOT CHANGE---MAX PHYSICAL HEIGHT
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
    public static final double GEAR_RATIO = 27/1.0;
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
