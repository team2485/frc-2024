package frc.robot.subsystems.drive;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.MedianFilter;

public class Drivetrain extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drive");
    GenericEntry absoluteGyroPos;
    GenericEntry currentGyroPos;

    private double absoluteGyroPostion = 0;

    public SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    public Pigeon2Configuration config = new Pigeon2Configuration();

    private MedianFilter filter = new MedianFilter(5);

    public Drivetrain() {
        //gyro.configFactoryDefault();
        absoluteGyroPos = Shuffleboard.getTab("Swerve").add("AbsoluteGyroOffset", 0).getEntry();
        currentGyroPos = Shuffleboard.getTab("Swerve").add("CurrentGyroOffset", 0).getEntry();
        gyro.reset();

        // mSwerveMods = new SwerveModule[] {
        //     new SwerveModule(0, Constants.Swerve.Mod0.constants),
        //     new SwerveModule(1, Constants.Swerve.Mod1.constants),
        //     new SwerveModule(2, Constants.Swerve.Mod2.constants),
        //     new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw().times(-1), getModulePositions());
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    -rotation, 
                                    Rotation2d.fromDegrees(gyro.getYaw().refresh().getValue() * -1)
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        absoluteGyroPos.setDouble(absoluteGyroPostion);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void driveWithSuppliedRotation(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Rotation2d absoluteRotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    -rotation, 
                                    absoluteRotation
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        absoluteGyroPos.setDouble(absoluteGyroPostion);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveAuto(ChassisSpeeds speeds) {
        driveWithSuppliedRotation(new Translation2d(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),-speeds.omegaRadiansPerSecond, true, false, getYawAbsolute());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    


    public double getPitch(){
        //return gyro.getRoll() + 4;
        return gyro.getPitch().refresh().getValue();
    }


    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = new SwerveModulePosition(mod.getPosition().distanceMeters, mod.getCanCoder());
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsInverted() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = new SwerveModulePosition(-mod.getPosition().distanceMeters, mod.getCanCoder());
        }
        return positions;
    }
    
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro(){
        absoluteGyroPostion += getYaw().times(-1).getDegrees();
        gyro.setYaw(0);
    }

    public void autoGyro(){
        gyro.setYaw(180);
    }

    
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(filter.calculate(360 - gyro.getYaw().refresh().getValue())) : Rotation2d.fromDegrees(filter.calculate(gyro.getYaw().refresh().getValue()));
    }

    public Rotation2d getYawAbsolute() {
        return Rotation2d.fromDegrees(absoluteGyroPostion).minus(getYaw()).times(-1);
    }

    public void resetToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
    
        }

    //     for(SwerveModule mod : mSwerveMods){
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        
    // }
    }