package frc.robot.subsystems.Vision;

import static frc.robot.Constants.PivotConstants.kPivotToRobot;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.BlueFieldConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RedFieldConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Interpolation.InterpolatingTable;
import frc.robot.commands.Interpolation.ShotCalculator;
import frc.robot.subsystems.drive.Drivetrain;

public class PoseEstimation extends SubsystemBase {
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotation;
  private final Supplier<SwerveModulePosition[]> modulePosition;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDrivePoseEstimator noVisionPoseEstimator;
  private final Field2d field2d = new Field2d();
  private final Vision photonEstimator = new Vision();
  private final Notifier photonNotifier = new Notifier(photonEstimator);
  private final WL_CommandXboxController m_driver;
    private final WL_CommandXboxController m_operator;

  private OriginPosition originPosition = OriginPosition.kRedAllianceWallRightSide;
  private boolean sawTag = false;
  private double angleToTags = 0;
  Supplier<ChassisSpeeds> speeds;

  ShotCalculator yawCalculator, pitchCalculator;

  Drivetrain m_drivetrain;

  GenericEntry visionTest;
  GenericEntry xSped;
  GenericEntry xLog;

  boolean isOnRed;

  public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition, Supplier<ChassisSpeeds> chassisSpeeds, WL_CommandXboxController m_driver, WL_CommandXboxController m_operator, Drivetrain m_drivetrain) {
    visionTest = Shuffleboard.getTab("Swerve").add("YSped", 10).getEntry();
    xSped = Shuffleboard.getTab("Swerve").add("XSped", 10).getEntry();
    xLog = Shuffleboard.getTab("Swerve").add("YDist", 0).getEntry();
    this.rotation = rotation;
    this.modulePosition = modulePosition;
    this.speeds = chassisSpeeds;
    this.m_driver = m_driver;
    this.m_operator = m_operator;

    poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.swerveKinematics,
        rotation.get(),
        modulePosition.get(),
        new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

    noVisionPoseEstimator = new SwerveDrivePoseEstimator(
        Swerve.swerveKinematics, 
        rotation.get(),
        modulePosition.get(), 
        new Pose2d());

    this.m_drivetrain = m_drivetrain;

    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);

    yawCalculator = new ShotCalculator();
    pitchCalculator = new ShotCalculator();

    isOnRed = getFieldConstants().isOnRed();
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
  }

  @Override
  public void periodic() {
    poseEstimator.update(rotation.get(), modulePosition.get());
    noVisionPoseEstimator.update(rotation.get(), modulePosition.get());
    var visionPose = photonEstimator.grabLatestEstimatedPose();
    if (visionPose != null) {
      var pose2d = visionPose.estimatedPose.toPose2d();
   
      poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
    }

    var dashboardPose = poseEstimator.getEstimatedPosition();
    // if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
    //   dashboardPose = flipAlliance(dashboardPose);
    // }
      
    field2d.setRobotPose(dashboardPose);
    angleToTags = getCurrentPose().getRotation().getDegrees();
    visionTest.setDouble(speeds.get().vyMetersPerSecond);
    xSped.setDouble((isOnRed?-1:1));
    
    //if (getNoteDetected()) m_operator.setRumble(RumbleType.kLeftRumble, 1);
    //else m_operator.setRumble(RumbleType.kLeftRumble, 0);
    
    

   // if (getNoteDetected()) m_driver.setRumble(RumbleType.kLeftRumble, 1);
    //else m_driver.setRumble(RumbleType.kLeftRumble, 0);
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  public Pose2d getCurrentVisionlessPose() {
    var pos = noVisionPoseEstimator.getEstimatedPosition();
    return pos;
  }

  public Pose2d getCurrentPose() {
    var pos = poseEstimator.getEstimatedPosition();
    
    if (pos.getX() < 0)
      pos = new Pose2d(new Translation2d(0, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    if (pos.getX() > VisionConstants.kFieldLengthMeters)
      pos = new Pose2d(new Translation2d(VisionConstants.kFieldLengthMeters, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    
    return pos;
  }

  public Pose2d getCurrentPoseNoVision() {
    return noVisionPoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public double getDistanceToSpeaker() {
    return dist(getFieldConstants().getSpeakerPos(), getCurrentPose());
  }

  public double dist(Pose2d pos1, Pose2d pos2) {
    double xDiff = pos2.getX()-pos1.getX();
    double yDiff = pos2.getY()-pos1.getY();
    return Math.sqrt(xDiff*xDiff + yDiff*yDiff);
  }

  public double getAngleToSpeaker() {
    double deltaY = getFieldConstants().getSpeakerPos().getY() - getCurrentPose().getY();
    double deltaX = getFieldConstants().getSpeakerPos().getX() - getCurrentPose().getX();
    Pose2d anglePos = new Pose2d(deltaX, deltaY, new Rotation2d());
    // return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
    return anglePos.getTranslation().getAngle().getDegrees();
  }

  public double getAngleFromTags() {
    if (angleToTags < -90) return 180 + angleToTags;
    if (angleToTags > 90) return -180 + angleToTags;
    return angleToTags;
  }

  public double getAngleToAmp() {
    return getAngleToPos(getFieldConstants().getAmpPos());
  }
  public double getAngleToStage() {
    //if (photonEstimator.grabLatestEstimatedPose() != null) 
    // int tagID = photonEstimator.grabLatestEstimatedPose().targetsUsed.get(0).getFiducialId();
    // Rotation3d angle = Constants.VisionConstants.kBlueTagList.get(tagID-1).pose.getRotation();
    // return angle.getAngle();
    return 0;
  }

  public double getAngleToPos(Pose2d pos) {
    double deltaY = pos.getY() - getCurrentPose().getY();
    double deltaX = pos.getX() - getCurrentPose().getX();
    return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
  }

  public boolean getNoteDetected() {
    return photonEstimator.getNoteExists();
  }

  public double getNoteAngle() {
    return photonEstimator.grabNoteYaw();
  }

  public double getNoteSetpoint() {
    return InterpolatingTable.getYaw(photonEstimator.grabNotePitch()).pivotAngleRotations;
  }

  public double getAngleToSpeakerCalculated() {
    //Translation2d futurePos = getCurrentPose().getTranslation().plus(new Translation2d(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond));
    //Translation2d pivotOffset = new Translation2d(kPivotToRobot*Math.cos(rotation.get().getRadians()), kPivotToRobot*Math.sin(rotation.get().getRadians()));
    yawCalculator.setPositions(getCurrentPose().getTranslation(), getFieldConstants().getSpeakerAnglePos().getTranslation());
    yawCalculator.setVelocities(-1*speeds.get().vxMetersPerSecond*(isOnRed?-1:1), 0, -1*speeds.get().vyMetersPerSecond*(isOnRed?-1:1));

    //v[0] = forward v[1] = vertical v[2] = horizontal
    double[] velocities = yawCalculator.shoot();
    double x = velocities[0];
    double y = velocities[2];
    double z = velocities[1];
    double theta = Rotation2d.fromRadians(Math.atan2(y,x)).getDegrees();
    return theta;

    // double angle = Rotation2d.fromRadians(yawCalculator.shootWhileMove()[0]).getRadians();
    // double angleX = -Math.cos(angle);
    // double angleY = -Math.sin(angle);
    // Translation2d reflectedVector = new Translation2d(angleX, angleY);

    // return Math.atan(reflectedVector.getY()/reflectedVector.getX());
  }

  public double getPivotAngleCalculated() {
    //Translation2d futurePos = getCurrentPose().getTranslation().plus(new Translation2d(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond));
    //Translation2d pivotOffset = new Translation2d(kPivotToRobot*Math.cos(rotation.get().getRadians()), kPivotToRobot*Math.sin(rotation.get().getRadians()));
    //visionTest.setDouble(pivotOffset.getX());
    pitchCalculator.setPositions(getCurrentPose().getTranslation(), getFieldConstants().getSpeakerPos().getTranslation());
    pitchCalculator.setVelocities(speeds.get().vxMetersPerSecond*1.5*(isOnRed?-1:1), 0, 1.5*speeds.get().vyMetersPerSecond*(isOnRed?-1:1));

    //v[0] = forward v[1] = vertical v[2] = horizontal
    double[] velocities = pitchCalculator.shoot();
    double x = velocities[0];
    double y = velocities[2];
    double z = velocities[1];
    double phi = 90-Rotation2d.fromRadians(Math.acos(z/Math.sqrt((x*x)+(y*y)+(z*z)))).getDegrees();
    double mappedPivotAngle = map(phi, 54, 0, 0, .139);
    return mappedPivotAngle;

    // double phi = Rotation2d.fromRadians(pitchCalculator.shootWhileMove()[1]).getRotations();
    // return phi;
  }

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public FieldConstants getFieldConstants() {
    RedFieldConstants redFieldConstants = new RedFieldConstants();
    BlueFieldConstants blueFieldConstants = new BlueFieldConstants();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return blueFieldConstants;
    }
    else {
      return redFieldConstants;
    }
  }
}