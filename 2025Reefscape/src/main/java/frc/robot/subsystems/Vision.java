package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class Vision extends SubsystemBase {
  //network tables
  private String hostname;
  private NetworkTable limelightTable;
  private double xOffset;
  private double verticalOffset;

  //fiducial
  private LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();
  private LimelightResults results = new LimelightResults();

  //misc
  private boolean isInRange = false;
  private Pose3d targetPose = new Pose3d();
  private ShuffleboardTab tab;
  
  /** Creates new Vision */
  public Vision(String hostname) {
    this.hostname = hostname;
    limelightTable = NetworkTableInstance.getDefault().getTable(hostname);
    targetPose = limelightTargetFiducial.getTargetPose_RobotSpace();

    tab = Shuffleboard.getTab("match");
    updateValues();
  }

  /** 
   * Update x offset value
   */
  public void updateValues() {
    xOffset = limelightTable.getEntry("tx").getDouble(0.0);
    verticalOffset = limelightTable.getEntry("ty").getDouble(0.0);
  }

  /**
   * Displays limelight camera view, xOffset value, and boolean showing whether limelight is in range on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab){
    tab.addCamera("limelight camera", "limelight", "http://10.19.67.11:5800/");
    tab.addDouble("x offset - left/right", () -> xOffset);
    tab.addDouble("y offset - top/bottom", () -> verticalOffset);
    tab.addBoolean("in range", () -> isInRange);
  }

  /**
   * Changes pipeline
   * @param isVision - if true, look for AprilTags
   */
  public void setVisionMode(boolean isVision){
    if (isVision) limelightTable.getEntry("pipeline").setNumber(0);
    else limelightTable.getEntry("pipeline").setNumber(1);
  }

  /** 
   * updates value of isInRange
   */
  public void alignAngleX(){
    updateValues();
    if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR){
      isInRange = false;
      tab.addBoolean("Range", () -> false);
    } else {
      isInRange = true;
      tab.addBoolean("Range", () -> true);
    }
  }

  /** @return whether limelight is in range */
  public boolean getIsInRange(){
    return false;
  }

  /** @return value of xOffset */
  public double getOffset() {
    return xOffset;
  }

  @Override
  public void periodic() {
    alignAngleX();
  }
}