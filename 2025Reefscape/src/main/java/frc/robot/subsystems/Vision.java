package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.commands.AlignBranch;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/

  // private boolean isRedAlliance;
  // public double verticalOffset, angleToGoalDegrees, angleToGoalRadians;
  // public double limelightToGoalInches = 0.0;
  private boolean disableVision = false;
  // private SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public NetworkTable limelightTable, limelightOdometryTable;
  public LimelightTarget_Fiducial limelightTargetFiducial = new LimelightTarget_Fiducial();

  public Pose3d targetPose = new Pose3d();
  public LimelightResults results = new LimelightResults();

  //Limelight Updating Values
  private double xAlignmentOffset, yAlignmentOffset, zAlignmentOffset, vAlignmentCheck;
  private double xOdometryOffset, yOdometryOffset, zOdometryOffset;
  private ChassisSpeeds alignSpeed;

  private boolean isInRange = false;
  
  /** Creates new Vision */
  public Vision() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    limelightOdometryTable = NetworkTableInstance.getDefault().getTable("limelight-santos");

    targetPose = limelightTargetFiducial.getTargetPose_RobotSpace();
    alignSpeed = new ChassisSpeeds();
    updateAlignmentValues();
    updateOdometryValues();

  }

  /** Update x offset value */
  public void updateAlignmentValues() {
    xAlignmentOffset = limelightTable.getEntry("tx").getDouble(0.0);
    yAlignmentOffset = limelightTable.getEntry("tz").getDouble(0.0);
    zAlignmentOffset = limelightTable.getEntry("ty").getDouble(0.0);
    vAlignmentCheck = limelightTable.getEntry("tv").getDouble(0.0);
    alignSpeed = AlignBranch.getAlignmentSpeed();
  }

  /** Update x offset value */
  public void updateOdometryValues() {
    xOdometryOffset = limelightOdometryTable.getEntry("tx").getDouble(0.0);
    yOdometryOffset = limelightOdometryTable.getEntry("tz").getDouble(0.0);
    zOdometryOffset = limelightOdometryTable.getEntry("ty").getDouble(0.0);
  }

  /**
   * Displays limelight camera view, xOffset value, and boolean showing whether limelight is in range on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab){
    //tab.addDouble("odo LL xOffset", () -> limelightOdometryTable.getEntry("tx")
   // .getDouble(0.0)).withWidget(BuiltInWidgets.kTextView)
   // .withPosition(0, 2).withSize(1, 1);

    tab.addDouble("odo LL yOffset", () -> limelightOdometryTable.getEntry("ty")
    .getDouble(0.0)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(1, 2).withSize(1, 1);

    tab.addBoolean("LL isDisabled", () -> isVisionDisabled())
    .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 2)
    .withSize(1, 1);
  }

  public void configLLTab(ShuffleboardTab tab) {
    HttpCamera httpCamera1 = new HttpCamera("limelight", "http://10.19.67.12:5801/"); //http://10.19.67.202:5801/
    CameraServer.addCamera(httpCamera1);

    HttpCamera httpCamera2 = new HttpCamera("limelight-santos", "http://10.19.67.11:5801/"); //http://10.19.67.11:5801/
    CameraServer.addCamera(httpCamera2);

    tab.add(httpCamera1).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
    .withSize(3, 2);
    tab.add(httpCamera2).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 0)
    .withSize(3, 2);
    tab.addBoolean("LL isInRange", () -> getInRange())
    .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 1)
    .withSize(1, 1);

    tab.addDouble("LL xOffset", () -> limelightTable.getEntry("tx")
    .getDouble(0.0)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(7, 0).withSize(1, 1);

    tab.addDouble("LL yOffset", () -> limelightTable.getEntry("ty")
    .getDouble(0.0)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(7, 1).withSize(1, 1);

    tab.addDouble("LL zOffset", () -> limelightTable.getEntry("tz")
    .getDouble(0.0)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(7, 2).withSize(1, 1);

    // tab.addDouble("LL alignmentSpeedY", () -> alignSpeed.vyMetersPerSecond)
    // .withPosition(7, 3).withSize(1, 1);

    // tab.addDouble("LL alignmentSpeedX", () -> alignSpeed.vxMetersPerSecond)
    // .withPosition(7, 4).withSize(1, 1);

    tab.addDouble("alignment z fiducial", () -> Constants.Vision.LIMELIGHT_ALIGN_Z_OFFSET).withWidget(BuiltInWidgets.kTextView);
  }

  /**
   * Changes pipeline
   * @param isVision - if true, look for AprilTags
   */
  public void setVisionMode(boolean isVision){
    if (isVision) limelightTable.getEntry("pipeline").setNumber(0);
    else limelightTable.getEntry("pipeline").setNumber(1);
  }

  /** @return value of xOffset */
  public double getTXAlignmentOffset() {
    return xAlignmentOffset;
  }

  public double getAlignmentCheck() {
    return vAlignmentCheck;
  }

  public double getZOffset() {
    return zAlignmentOffset;
  }

  public double getYOffset(){
    return yAlignmentOffset;
  }

  public void disableVision(){
    disableVision = !disableVision;
  }

  /**
   * @return true when disabled
   */
  public boolean isVisionDisabled(){
    return disableVision;
  }

  public boolean getInRange() {
    return isInRange;
  }

  public void setInRangeTrue() {
    isInRange = true;
  }

  public void setInRangeFalse() {
    isInRange = false;
  }

  // /** updates value of isInRange */
  // public void alignAngleX(){
  //   updateAlignmentValues();
  //   if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR){
  //     isInRange = false;
  //   } else {
  //     isInRange = true;
  //   }
  // }

  // public void alignAngleZ(){
  //   updateValues();
  //   angleToGoalDegrees = Constants.Vision.LIMELIGHT_ANGLE_DEGREES + verticalOffset;
  //   angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
  //   limelightToGoalInches = (Constants.Vision.TARGET_HEIGHT_INCHES - Constants.Vision.LIMELIGHT_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
  // }


  // public void onEnable(Optional<Alliance> alliance){
  //   if (alliance.get() == Alliance.Red) isRedAlliance = true;
  //   else isRedAlliance = false;
  // }
  //  public boolean getAlliance(){
  //    return isRedAlliance;
  //  }

  //  //NEW
  //  public Rotation3d getBlueFieldRot(){
  //   return results.getBotPose3d_wpiBlue().getRotation(); //yaw?
  //   //return (limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[5]);
  // }

  // public double getBlueFieldX() {
  //   //return results.getBotPose3d_wpiBlue().getX();
  //   return LimelightHelpers.pose3dToArray(results.getBotPose3d_wpiBlue())[0];
  //   //(limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0]);
  // }

  // public double getBlueFieldY() {
  //   return results.getBotPose3d_wpiBlue().getY();
  //   //(limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[1]);
  // }

  @Override
  public void periodic() {
    updateAlignmentValues();
    updateOdometryValues();
  }
}