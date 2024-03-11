package frc.robot.subsystems.Vision;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlueFieldConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RedFieldConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Interpolation.ShotCalculator;

public class PoseEstimation extends SubsystemBase {
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotation;
  private final Supplier<SwerveModulePosition[]> modulePosition;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final Vision photonEstimator = new Vision();
  private final Notifier photonNotifier = new Notifier(photonEstimator);

  private OriginPosition originPosition = OriginPosition.kRedAllianceWallRightSide;
  private boolean sawTag = false;
  private double angleToTags = 0;
  Supplier<ChassisSpeeds> speeds;

  GenericEntry visionTest;

  public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition, Supplier<ChassisSpeeds> chassisSpeeds) {
    visionTest = Shuffleboard.getTab("Swerve").add("RequestedAngle", 0).getEntry();
    this.rotation = rotation;
    this.modulePosition = modulePosition;
    this.speeds = chassisSpeeds;

    poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.swerveKinematics,
        rotation.get(),
        modulePosition.get(),
        new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
  }

  @Override
  public void periodic() {
    poseEstimator.update(rotation.get(), modulePosition.get());
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
    visionTest.setDouble(getAngleToSpeakerCalculated());
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  public Pose2d getCurrentPose() {
    var pos = poseEstimator.getEstimatedPosition();
    if (pos.getX() < 0)
      pos = new Pose2d(new Translation2d(0, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    if (pos.getX() > VisionConstants.kFieldLengthMeters)
      pos = new Pose2d(new Translation2d(VisionConstants.kFieldLengthMeters, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    
    return pos;
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

  public double getAngleToPos(Pose2d pos) {
    double deltaY = pos.getY() - getCurrentPose().getY();
    double deltaX = pos.getX() - getCurrentPose().getX();
    return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
  }

  public double getAngleToSpeakerCalculated() {
    ShotCalculator.setPositions(getCurrentPose().getTranslation(), getFieldConstants().getSpeakerAnglePos().getTranslation());
    ShotCalculator.setVelocities(0, 0, 0);
    // v[0] = forward v[1] = vertical v[2] = horizontal
    double[] velocities = ShotCalculator.shoot();
    double x = velocities[0];
    double y = velocities[2];
    double z = velocities[1];
    double theta = Rotation2d.fromRadians(Math.atan2(y,x)).getDegrees();
    return theta;
  }

  public double getPivotAngleCalculated() {
    ShotCalculator.setPositions(getCurrentPose().getTranslation(), getFieldConstants().getSpeakerPos().getTranslation());
    ShotCalculator.setVelocities(0, 0, 0);
    // v[0] = forward v[1] = vertical v[2] = horizontal
    double[] velocities = ShotCalculator.shoot();
    double x = velocities[0];
    double y = velocities[2];
    double z = velocities[1];
    double phi = 90-Rotation2d.fromRadians(Math.acos(z/Math.sqrt((x*x)+(y*y)+(z*z)))).getDegrees();
    double mappedPivotAngle = map(phi, 54, 0, 0, .139);
    return mappedPivotAngle;
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