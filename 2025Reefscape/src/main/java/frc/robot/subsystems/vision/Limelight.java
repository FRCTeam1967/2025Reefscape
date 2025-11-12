package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.LimelightSubsystem;
import frc.lib.io.VisionIOLimelight;
import frc.robot.subsystems.drive.Drive;

public class Limelight extends LimelightSubsystem<VisionIOLimelight> {
	public static final Limelight mInstance = new Limelight();

	private Pose2d lastPose = new Pose2d(); // stores most recent drivetrain pose
	private long numPoseStableUpdates = 0; // count of vision updates that agree with drivetrain pose

	private Limelight() {
		super(LimelightConstants.getVisionIOConfig(), LimelightConstants.getVisionIO());
	}

	@Override
	public void periodic() {
		try {
			super.periodic();
			Pose2d ioPose = io.getLatestEstimate(); // gets latest vision pose
			if (ioPose != lastPose) { // compares limelight pose to the drivetrain pose
				if (Drive.mInstance.getPose().getTranslation().getDistance(ioPose.getTranslation())
						< LimelightConstants.agreedTranslationUpdateEpsilon.in(Units.Meters)) {
					numPoseStableUpdates++; // if the difference in translation of the limelight pos and drivetrain pos is less than a certain constant, accept pose update
				} else {
					numPoseStableUpdates = 0;
				}
			}

			lastPose = ioPose;

			SmartDashboard.putNumber("Vision/Num Agreed Stable Updates", numPoseStableUpdates);
		} catch (Exception e) { // if you cannot update pose then log the crash
			SmartDashboard.putNumber("Limelight/Crash", Timer.getFPGATimestamp());
			SmartDashboard.putString("Limelight/Crash Exception", e.getMessage());
			SmartDashboard.putString(
					"Limelight/Crash Stacktrace", e.getStackTrace().toString());
		}
	}

	public Time getLastUpdateTime() {
		return io.getLatestEstimateTime();
	}

	public Pose2d getLatestUpdate() {
		return lastPose;
	}

	public boolean getPoseStable() { // return if pose is stable (more than 100 updates)
		return numPoseStableUpdates > LimelightConstants.agreedTranslationUpdatesThreshold;
	}
}
