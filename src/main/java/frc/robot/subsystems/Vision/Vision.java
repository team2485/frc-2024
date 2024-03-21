package frc.robot.subsystems.Vision;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.VisionConstants.*;

public class Vision implements Runnable {
    // PhotonVision class that takes vision measurements from camera and triangulates position on field
    private final PhotonPoseEstimator m_photonPoseEstimator;
    // creates new PhotonCamera object for camera
    private final PhotonCamera m_camera; // initialize with a USEFUL name;
    private final PhotonCamera m_noteCamera;
    // creates a thread-safe object reference since mutliple robot poses could be reported concurrently and conflict
    // lowkey an interesting read, the atomic library gaslights machine instruction algos like compare-and-swap
    // that are instrinsically atomic into working for concurrency
    GenericEntry cameraExists;
    private final AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    private double notePitch = 0;
    private double noteYaw = 0;
    private boolean noteExists = false;

    public Vision() {
        PhotonPoseEstimator photonPoseEstimator = null;

        cameraExists = Shuffleboard.getTab("Swerve").add("CameraExists", 0).getEntry();
    

        this.m_camera = new PhotonCamera(kCameraName);
        this.m_noteCamera = new PhotonCamera("notecamera");

        try {
            //sets the origin to the blue side every time but flips the tag positions if we are red.
            var layout = new AprilTagFieldLayout(kBlueTagList, kFieldLengthMeters, kFieldWidthMeters); // attempt to load the AprilTagFieldLayout
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            if (m_camera != null) {
                photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera,
                        kRobotToCamera); // MULTI_TAG_PNP uses all cameras in view for positioning
            }
        } catch (Exception e) {
            DriverStation.reportError("Path: ", e.getStackTrace()); // can't estimate poses without known tag positions
            photonPoseEstimator = null;
        }

        this.m_photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        if (m_photonPoseEstimator != null && m_camera != null) { // environment and camera must be initialized properly
            var photonResults = m_camera.getLatestResult(); // continuously get latest camera reading
            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1 || (photonResults.targets.get(0).getPoseAmbiguity()<.15 && photonResults.targets.get(0).getPoseAmbiguity() > 0))) { // need accurate readings       
                m_photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;

                    SmartDashboard.putBoolean("Camera Positioned For Auto", true);
                    //cameraExists.setDouble(photonResults.targets.get(0).getBestCameraToTarget().getX()); 
                    //cameraExists.setDouble(photonResults.getMultiTagResult().estimatedPose.best.getX());

                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= kFieldLengthMeters
                            && estimatedPose.getY() > 0.0
                            && estimatedPose.getY() <= kFieldWidthMeters) { // idiot check
                        m_atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
            else SmartDashboard.putBoolean("Camera Positioned For Auto", false);
        }
        if (m_noteCamera != null) {
            var photonResults = m_noteCamera.getLatestResult();
            if (photonResults.hasTargets()) {
                photonResults.targets.removeIf(n -> (n.getPitch() > 0));
                if (photonResults.targets.size() > 0) {
                    noteExists = true;
                    notePitch = photonResults.targets.get(0).getPitch();
                    noteYaw = photonResults.targets.get(0).getYaw();
                    cameraExists.setDouble(grabNoteYaw());
                }
                else noteExists = false;
            }
            else noteExists = false;
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return m_atomicEstimatedRobotPose.get();
    }

    public double grabNotePitch() {
        return notePitch;
    }

    public double grabNoteYaw() {
        return noteYaw;
    }

    public boolean getNoteExists() {
        return noteExists && notePitch < 0;
    }
}