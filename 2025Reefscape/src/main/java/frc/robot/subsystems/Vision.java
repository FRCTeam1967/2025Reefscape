package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/
  private NetworkTable limelightTable;
  private double xOffset = -100000;
  private boolean isInRange = false;
  private boolean isRedAlliance;
  public double verticalOffset, angleToGoalDegrees, angleToGoalRadians;
  public double limelightToGoalInches = 0.0;
  private String limelightHostname;

  /** Creates new Vision */
  public Vision(String hostname) {
    limelightHostname = hostname;
    limelightTable = NetworkTableInstance.getDefault().getTable(limelightHostname);
    updateValues();
  }

  /** Update x offset value */
  public void updateValues() {
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    xOffset = tx.getDouble(0.0);
    verticalOffset = ty.getDouble(0.0);
    SmartDashboard.putNumber("X Offset", xOffset);
    SmartDashboard.putNumber("Y Offset", verticalOffset);
  }

  /**
   * Displays limelight camera view, xOffset value, and boolean showing whether limelight is in range on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab){
    tab.addCamera("Limelight Camera", "m_limelight", "http://10.19.67.11:5800/");
    tab.addDouble("Limelight xOffset", () -> limelightTable.getEntry("tx").getDouble(0.0));
    tab.addDouble("Limelight yOffset", () -> limelightTable.getEntry("ty").getDouble(0.0));
    tab.addBoolean("In Range", ()->isInRange);
    tab.addDouble("Distance to Target", () -> limelightToGoalInches);
  }

  /**
   * Changes pipeline
   * @param isVision - if true, look for AprilTags
   */
  public void setVisionMode(boolean isVision){
    if (isVision) limelightTable.getEntry("pipeline").setNumber(0);
    else limelightTable.getEntry("pipeline").setNumber(1);
  }

  /** updates value of isInRange */
  public void alignAngleX(){
    updateValues();
    if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR){
      isInRange = true;
      SmartDashboard.putString("Range", "yes");
    } else {
      isInRange = false;
      SmartDashboard.putString("Range", "yes");
    }
  }

  public void alignAngleZ(){
    updateValues();
    angleToGoalDegrees = Constants.Vision.LIMELIGHT_ANGLE_DEGREES + verticalOffset;
    angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    limelightToGoalInches = (Constants.Vision.TARGET_HEIGHT_INCHES - Constants.Vision.LIMELIGHT_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
  }

  /** @return whether limelight is in range */
  public boolean getIsInRange(){
    return isInRange;
  }

  /** @return value of xOffset */
  public double getOffset() {
    return xOffset;
  }

  public void onEnable(Optional<Alliance> alliance){
    if (alliance.get() == Alliance.Red) isRedAlliance = true;
    else isRedAlliance = false;
  }
   public boolean getAlliance(){
     return isRedAlliance;
   }

  @Override
  public void periodic() {
    alignAngleX();
    alignAngleZ();
  }
}