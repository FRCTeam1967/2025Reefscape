package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.LimelightSubsystem.LimelightConfig;
import frc.lib.io.VisionIOLimelight;
import frc.lib.io.VisionIOLimelightSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class LimelightConstants {
	public static final String kLimelightName = "limelight";

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1;
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

	public static Pose3d kRobotToCameraOffset;

	/**
	 * use cad measurements as baseline for limelight pos to robot
	 * set up center of robot is at center of tag (known robot pos)
	 * limelight calculate robot pose -> tune for actual pose
	 * change rotation before translation
	 */ 
	static {
		if (RobotConstants.isOmega) {
			kRobotToCameraOffset = new Pose3d(
					new Translation3d(Units.Inches.of(-5.344), Units.Inches.of(9.044), Units.Inches.of(8.438)),
					new Rotation3d(Units.Degree.of(0), Units.Degree.of(-15.0), Units.Degree.of(160.0))); //set phone against ll with angle measurement thing
					// pitch downward is negative, yaw clockwise is positive? "signs are weird"
		} else {
			kRobotToCameraOffset = new Pose3d(
					new Translation3d(Units.Inches.of(-3.22), Units.Inches.of(9.443), Units.Inches.of(9.127)),
					new Rotation3d(Units.Degree.of(0), Units.Degree.of(-12.0), Units.Degree.of(156.0)));
		}
	}

	public static final LimelightConfig getVisionIOConfig() {
		LimelightConfig config = new LimelightConfig();
		config.name = kLimelightName;
		config.robotToCameraOffset = kRobotToCameraOffset;
		return config;
	}

	public static final VisionIOLimelight getVisionIO() {
		if (Robot.isReal()) {
			return new VisionIOLimelight();
		} else {
			return new VisionIOLimelightSim();
		}
	}

	public static final int agreedHeadingUpdatesThreshold = 100;
	public static final Angle agreedHeadingUpdateEpsilon = Units.Degrees.of(2.0);
	public static final int agreedTranslationUpdatesThreshold = 100;
	public static final Distance agreedTranslationUpdateEpsilon = Units.Centimeters.of(10.0);
}
