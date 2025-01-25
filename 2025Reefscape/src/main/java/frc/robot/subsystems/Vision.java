package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/

  private NetworkTable limelightTable;
  private double xOffset = -100000;
  private boolean isInRange = false;
  private boolean isRedAlliance;
  public double verticalOffset, angleToGoalDegrees, angleToGoalRadians;
  public double limelightToGoalInches = 0.0;
  private String limelightHostname;
  public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();
  

  public Pose3d targetPose = new Pose3d();
  public LimelightResults results = new LimelightResults();
  
  
  
  /** Creates new Vision */
  public Vision(String hostname) {
    limelightHostname = hostname;
    limelightTable = NetworkTableInstance.getDefault().getTable(limelightHostname);
    targetPose = limelightTargetFiducial.getTargetPose_RobotSpace();
    updateValues();
  }

  
  /** Update x offset value */
  public void updateValues() {

    //targetPose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(targetPose);
    //double xOffset = targetPose.getX();
    //double verticalOffset = targetPose.getY();

    xOffset = limelightTable.getEntry("tx").getDouble(0.0);
    verticalOffset = limelightTable.getEntry("ty").getDouble(0.0);

    SmartDashboard.putNumber("X Offset", xOffset);
    SmartDashboard.putNumber("Y Offset", verticalOffset);
    SmartDashboard.putNumber("Blue X", getBlueFieldX());
    SmartDashboard.putNumber("Blue Y", getBlueFieldY());
    SmartDashboard.putNumber("Blue ROT", getBlueFieldRot().getAngle());
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
      isInRange = false;
      SmartDashboard.putBoolean("Range", false);
    } else {
      isInRange = true;
      SmartDashboard.putBoolean("Range", true);
      
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
    return false;
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


   //NEW
   public Rotation3d getBlueFieldRot(){
    return results.getBotPose3d_wpiBlue().getRotation(); //yaw?
    //return (limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[5]);
  }

  public double getBlueFieldX() {
    //return results.getBotPose3d_wpiBlue().getX();
    return LimelightHelpers.pose3dToArray(results.getBotPose3d_wpiBlue())[0];
    //(limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0]);
  }

  public double getBlueFieldY() {
    return results.getBotPose3d_wpiBlue().getY();
    //(limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[1]);
  }

  @Override
  public void periodic() {
    alignAngleX();
    alignAngleZ();
  
    //m_field.setRobotPose(1.0,1.0, new Rotation2d(.5));
  
  }
}