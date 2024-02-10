package frc.robot.subsystems.Vision;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  GenericEntry visionTest;

  public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition) {
    visionTest = Shuffleboard.getTab("Swerve").add("XDist", 0).getEntry();
    this.rotation = rotation;
    this.modulePosition = modulePosition;

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

    visionTest.setDouble(getDistanceToSpeaker());
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public double getDistanceToSpeaker() {
    return dist(getFieldConstants().getSpeakerPos().getX(), getFieldConstants().getSpeakerPos().getY(), getCurrentPose().getX(), getCurrentPose().getY());
  }

  public double dist(double x1, double y1, double x2, double y2) {
    double xDiff = x2-x1;
    double yDiff = y2-y1;
    return Math.sqrt(xDiff*xDiff + yDiff*yDiff);
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