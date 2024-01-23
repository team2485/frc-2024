package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import static frc.robot.Constants.Swerve.*;

public class DriveCommandBuilder {
    public static Command driveToPosition(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation, Supplier<Pose2d> fieldEndPos) {
    
        AutoBuilder.configureHolonomic(
            m_poseEstimation::getCurrentPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::driveAuto, 
            kPathFollowingConfig, 
            () -> false , 
            m_drivetrain);
    
        
    
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = fieldEndPos.get();

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                kTeleopMaxSpeedMetersPerSecond, kTeleopMaxAccelerationMetersPerSecondSquared,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);


        // PathFindHolonomic is confirmed functional without collisions avoidance, AutoBuilder must be used to avoid collision
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // Command pathfindingCommand = new PathfindHolonomic(
        //                                 targetPose,
        //                                 constraints,
        //                                 0.0, // Goal end velocity in m/s. Optional
        //                                 m_poseEstimation::getCurrentPose,
        //                                 m_drivetrain::getChassisSpeeds,
        //                                 m_drivetrain::driveAuto,
        //                                 kPathFollowingConfig, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
        //                                 0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
        //                                 m_drivetrain // Reference to drive subsystem to set requirements
        //                                 );
        // return pathfindingCommand;

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,   
            0,
            0);
        return pathfindingCommand;
    }
}
