// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.Map;
import java.util.TreeMap;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Include units in Constant names whenever possible/convenient. If not, include a comment
 * designating units.
 */
public final class Constants {
  public static final String kRobotIdFile = "/home/lvuser/id.txt";
  public static final String kCurrentLogFolder = "/home/lvuser/currentLogs";
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final double kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;
  public static final double kSecondsPer100Ms = 0.1;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

  // 5 ft front bumper: 60 0.8

  public static final double kShootingFenderSetpointShooter = 24;
  public static final double kShootingFenderSetpointTangentialRatio = 0.55;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.05;
    public static final double kDriverLeftXDeadband = 0.05;
    public static final double kDriverLeftYDeadband = 0.05;

    public static final double kTriggerThreshold = 0.1;
  }

  public static final class AutoConstants {
    public static final double kAutoMaxSpeedMetersPerSecond = 1.5;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        1.5 / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 1 * Math.PI;

    public static final double kPAutoXController = 5;
    public static final double kIAutoXController = 0;
    public static final double kDAutoXController = 0;
    public static final double kPAutoYController = 5;
    public static final double kIAutoYController = 0;
    public static final double kDAutoYController = 0;

    public static final double kAutoXYIntegratorMaxMetersPerSecond = 0.5;
    public static final double kPAutoThetaController = 1;
    public static final double kIAutoThetaController = 0;
    public static final double kAutoThetaIntegratorMaxRadiansPerSecond = 0.2;
    public static final double kDAutoThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kAutoMaxAngularSpeedRadiansPerSecond,
            kAutoMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ModuleConstants {
    // Drive control constants
    public static final double kDriveSupplyCurrentLimitAmps = 35;
    public static final double kDriveStatorCurrentLimitAmps = 60;
    public static final double kDriveStatorCurrentThresholdTimeSecs = 0.1;

    //// Drive mechanism/encoder constants
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = 0.1016 * Math.PI;
    public static final double kDriveGearRatio = 6.86; // motor turns per wheel turns
    public static final double kDriveDistMetersPerMotorRev =
        kWheelCircumferenceMeters / kDriveGearRatio;
    public static final double kDriveDistMetersPerPulse =
        kDriveDistMetersPerMotorRev / kFalconSensorUnitsPerRotation;
    //// NOTE: CTRE Encoders return velocity in units/100 ms. CTRE velocity readings should be
    // multiplied by 10 to be per second.
    //// Drive feedforward constants
    // Field Carpet characterization constants
    // public static final double ksDriveVolts = 0.66707;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7887;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.29537;

    // Practice carpet characterization constants
    // public static final double ksDriveVolts = 0.667;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7695;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.23776;

    // practice carpet
    public static final double ksDriveVolts = 0.51019;
    public static final double kvDriveVoltSecondsPerMeter = 2.2644;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.1;

    public static final double ksDriveVoltsBackLeft = 0.5;

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.1;
    // Turning control constants
    public static final double kTurningSupplyCurrentLimitAmps = 20;
    public static final double kTurningStatorCurrentLimitAmps = 60;
    public static final double kTurningStatorCurrentThresholdTimeSecs = 0.1;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse =
        kTurningRadiansPerMotorRev / kFalconSensorUnitsPerRotation;

    //// Turning feedforward constants
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerRadian = 0.20175;
    public static final double kaTurningVoltSecondsSquaredPerRadian = 0.0053;

    //// Turning PID constants
    public static final double kPTurningOutputUnit100MsPerSensorUnit =
        1.5 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kDTurningOutputUnit100MsSquaredPerSensorUnit =
        0.2 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kFTurningOutputUnit100MsPerSensorUnit = 0.4 * 1023 / 8360;

    public static final double kTurningPositionToleranceSensorUnits =
        Units.degreesToRadians(2) * kFalconSensorUnitsPerRotation;
    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 8 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 64 * Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms =
        kModuleMaxSpeedTurningRadiansPerSecond / kTurningRadiansPerPulse * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared =
        kModuleMaxAccelerationTurningRadiansPerSecondSquared / kTurningRadiansPerPulse * 0.01;
  }

  public static final class DriveConstants {
    // Ports and zeros
    /** Zeros found with bevel gears facing right. Applied offset is the negative of the zero. */
    public static final int kPigeonPort = 9;

    public static final boolean kDriveInverted = true;
    public static final int kFLDriveTalonPort = 5;
    public static final int kFLTurningTalonPort = 6;
    public static final int kFLCANCoderPort = 12;
    // public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(56.0 - 1.66);
    public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(47.96);

    public static final int kFRDriveTalonPort = 7;
    public static final int kFRTurningTalonPort = 8;
    public static final int kFRCANCoderPort = 13;
    // public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-85.48 - 135);
    public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(136.48 - 180);

    public static final int kBRDriveTalonPort = 1;
    public static final int kBRTurningTalonPort = 2;
    public static final int kBRCANCoderPort = 10;
    // public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(-3.6 - 5.18);
    public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(170.89);

    public static final int kBLDriveTalonPort = 3;
    public static final int kBLTurningTalonPort = 4;
    public static final int kBLCANCoderPort = 11;
    // public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(-167.7 + 4.33 -
    // 3.06);
    public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(-168.98);

    // Drivebase dimensions
    public static final double kWheelbaseLengthMeters = 0.635; // meters
    public static final double kWheelbaseWidthMeters = 0.508; // meters

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelbaseLengthMeters / 2, kWheelbaseWidthMeters / 2),
            new Translation2d(kWheelbaseLengthMeters / 2, -kWheelbaseWidthMeters / 2),
            new Translation2d(-kWheelbaseLengthMeters / 2, kWheelbaseWidthMeters / 2),
            new Translation2d(-kWheelbaseLengthMeters / 2, -kWheelbaseWidthMeters / 2));

    public static final double kTurningRadiusMeters =
        Math.sqrt(Math.pow(kWheelbaseLengthMeters / 2, 2) + Math.pow(kWheelbaseWidthMeters / 2, 2));

    // Max speed teleoperated
    public static final double kTeleopMaxSpeedMetersPerSecond = 3; // meters per second
    public static final double kTeleopMaxAngularSpeedRadiansPerSecond =
        1.5 / kTurningRadiusMeters; // radians per second

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared = 1.5 * Math.PI;

    public static final int kPoseHistoryCapacity = 500;

    public static final double kPRotation = 2;
    public static final double kRotationTolerance = 0.05;
  }

  public static final class FieldConstants {
    // ALL CONSTANTS IN METERS

    public static final double kVisionTargetHeightLower =
        Units.inchesToMeters(8 * 12 + 5.625); // Bottom of tape

    public static final double kVisionTargetHeightUpper =
        kVisionTargetHeightLower + Units.inchesToMeters(2);

    public static final double kVisionTargetRadius = Units.inchesToMeters(4.0 * 12.0 + 5.375) / 2;
    // All coordinates start at blue terminal corner (0,0)
    // Driver station to driver station is x

    public static final Translation2d kHubCenterTranslation = new Translation2d(8.2296, 4.1148);
    public static final Pose2d kHubCenterPosition =
        new Pose2d(kHubCenterTranslation, new Rotation2d(0));

    public static final double kRobotBumperLengthMeters = 0.97;
    public static final double kRobotBumperWidthMeters = 0.84;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "photonvision";

    // old constraints (might want to use again)

    public static final TrapezoidProfile.Constraints kXConstraints = new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints kYConstraints = new TrapezoidProfile.Constraints(.5, 2);
    public static final TrapezoidProfile.Constraints kOmegaConstraints = new TrapezoidProfile.Constraints(3, 8);

    public static final double kTranslationTolerance = 0.02;
    public static final double kThetaTolerance = Units.degreesToRadians(0);

    // TODO: tune!
    public static final TrapezoidProfile.Constraints kDefaultXYContraints = new TrapezoidProfile.Constraints(
        Swerve.maxSpeed * 0.3,
        Swerve.maxAngularVelocity);

    public static final TrapezoidProfile.Constraints kDefaultOmegaConstraints = new TrapezoidProfile.Constraints(
        Swerve.maxAngularVelocity * 0.2,
        Swerve.maxAngularVelocity);

    // TODO: tune!
    public static final double X_kP = 1.25;
    public static final double X_kI = 0.3;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 1.25;
    public static final double Y_kI = 0.3;
    public static final double Y_kD = 0.0;  

    public static final double THETA_kP = 1.5;
    public static final double THETA_kI = 0.5;
    public static final double THETA_kD = 0.15;

    // TODO: ensure validity of measurements
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(-.33, 0, .2667),
        new Rotation3d());

    public static final double kFieldLengthMeters = 16.54175;
    public static final double kFieldWidthMeters = 8.0137;

    public static final Pose2d kFlippingPose = new Pose2d(
        new Translation2d(kFieldLengthMeters, kFieldWidthMeters),
        new Rotation2d(Math.PI));

    // public static final int kTagOfInterest = 1;
    // public static final Transform2d kTagToGoal = new Transform2d(new
    // Translation2d(1, 0),
    // Rotation2d.fromDegrees(180.0));


    public static final double kOffsetToNextScoringStation = 0.61;

    public static final double kTopTagYPos = kFieldWidthMeters - 4.42;
    public static final double kMiddleTagYPos = kFieldWidthMeters - 2.75;
    public static final double kBottomTagYPos = kFieldWidthMeters - 1.07;

  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 14;
    public static final int kIntakeCurrentLimit = 60;
    public static final boolean kIntakeInverted = false;
    public static final double kIntakeKp = .012;
    public static final double kIntakeKi = 0;
    public static final double kIntakeKd = 0;
  }

  public static final class IndexerConstants {
  
  }

  public static final class FeederConstants {
    public static final int kFeederTalonPort = 16;
    public static final double kFeederLoopTimeSeconds = 0.02;
    public static final int kFeederSmartCurrentLimitAmps = 15;
    public static final int kFeederImmediateCurrentLimitAmps = 20;

    public static final double kFeederSupplyCurrentLimitAmps = 25;
    public static final double kFeederSupplyCurrentThresholdAmps = 30;
    public static final double kFeederSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kFeederStatorCurrentLimitAmps = 40;
    public static final double kFeederStatorCurrentThresholdAmps = 45;
    public static final double kFeederStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kFeederGearRatio = 1; // motor turns : output/full hood turns

    public static final double kFeederFreeSpeedRotationsPerSecond =
        kNeo550FreeSpeedRotationsPerSecond / kFeederGearRatio;

    public static final double kFeederDefaultSpeedRotationsPerSecond =
        kFeederFreeSpeedRotationsPerSecond * 0.75;

    public static final double kFeederPulleyCircumferenceMeters = 0.0191008 * Math.PI;

    public static final double kFeederSurfaceFreeSpeedMetersPerSecond =
        kFeederFreeSpeedRotationsPerSecond * kFeederPulleyCircumferenceMeters;

    public static final double kFeederShooterSurfaceSpeedRatio =
        kFeederSurfaceFreeSpeedMetersPerSecond
            / ShooterConstants.kShooterSurfaceFreeSpeedMetersPerSecond;

    public static final double kSFeederVolts = 0.56782;
    public static final double kVFeederVoltSecondsPerMeter = 0.6591;
    public static final double kAFeederVoltSecondsSquaredPerMeter = 0.032787;

    public static final double kFeederVelocityToleranceRotationsPerSecond = 1;

    public static final int kFeederServoPort = 1;
    public static final double kServoDisengagedPosition = 0.43;
    public static final double kServoEngagedPosition = 0.19;
  }

  public static final class HoodConstants {
    public static final int kHoodSparkPort = 23;
    public static final double kHoodGearRatio = 225; // motor turns : output/full hood turns
    public static final double kHoodRadiansPerMotorRev = 2 * Math.PI / kHoodGearRatio;

    public static final double kHoodBottomPositionRadians = 0; // from horizontal
    public static final double kHoodTopPositionRadians = 0.2872;

    public static final int kHoodSmartCurrentLimitAmps = 10;
    public static final int kHoodImmediateCurrentLimitAmps = 10;

    // Hood characterization constants
    public static final double kSHoodVolts = 0.1;
    public static final double kGHoodVolts = 0.25;
    public static final double kVHoodVoltSecondsPerRadian = 1;
    public static final double kAHoodVoltSecondsSquaredPerRadian = 0.12369;

    public static final double kHoodMaxSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kHoodMaxAccelerationRadiansPerSecondSquared =
        new ArmFeedforward(
                kSHoodVolts,
                kGHoodVolts,
                kVHoodVoltSecondsPerRadian,
                kAHoodVoltSecondsSquaredPerRadian)
            .maxAchievableAcceleration(
                kNominalVoltage, kHoodBottomPositionRadians, kHoodBottomPositionRadians);

    public static final TrapezoidProfile.Constraints kHoodMotionProfileConstraints =
        new TrapezoidProfile.Constraints(
            kHoodMaxSpeedRadiansPerSecond, kHoodMaxAccelerationRadiansPerSecondSquared);
    // Hood PID constants
    public static final double kPHood =  10;
    public static final double kIHood = .1;
    public static final double kDHood = 0;
    public static final double kHoodControllerPositionTolerance = 0.005;
  }

  public static final class ShooterConstants {
    public static final int kShooterTalonPort1 = 17;
    public static final int kShooterTalonPort2 = 19;

    public static final double kShooterSupplyCurrentLimitAmps = 25;
    public static final double kShooterSupplyCurrentThresholdAmps = 30;
    public static final double kShooterSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kShooterStatorCurrentLimitAmps = 40;
    public static final double kShooterStatorCurrentThresholdAmps = 45;
    public static final double kShooterStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kKickerSupplyCurrentLimitAmps = 40;
    public static final double kKickerSupplyCurrentThresholdAmps = 45;
    public static final double kKickerSupplyCurrentThresholdTimeSecs =
        kShooterSupplyCurrentLimitAmps;
    public static final double kKickerStatorCurrentLimitAmps = 60;
    public static final double kKickerStatorCurrentThresholdAmps = 65;
    public static final double kKickerStatorCurrentThresholdTimeSecs =
        kShooterStatorCurrentThresholdTimeSecs;

    public static final double kShooterLoopTimeSeconds = 0.001;
    public static final double kKickerLoopTimeSeconds = kShooterLoopTimeSeconds;

    public static final double kShooterGearRatio = 1;
    public static final double kKickerGearRatio = 0.5;

    public static final double kShooterCircumferenceMeters =
        0.1524 * Math.PI; // 6 in diameter wheel
    public static final double kKickerCircumferenceMeters = 0.0508 * Math.PI; // 2 in diameter wheel

    public static final double kShooterFreeSpeedRotationsPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kShooterGearRatio;
    public static final double kKickerFreeSpeedRotationsPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kKickerGearRatio;

    public static final double kShooterMaxSpeedRotationsPerSecond = 68; // empirical estimate
    public static final double kKickerMaxSpeedRotationsPerSecond = 180; // empirical estimate

    public static final double kShooterSurfaceFreeSpeedMetersPerSecond =
        kShooterFreeSpeedRotationsPerSecond * kShooterCircumferenceMeters;
    public static final double kKickerSurfaceFreeSpeedMetersPerSecond =
        kKickerFreeSpeedRotationsPerSecond * kKickerCircumferenceMeters;

    public static final double kSShooterVolts = 0.25;
    public static final double kVShooterVoltSecondsPerRotation = 0.133;
    public static final double kAShooterVoltSecondsSquaredPerRotation = 0.005;

    public static final double kSKickerVolts = 0.5;
    public static final double kVKickerVoltSecondsPerRotation = 0.1;
    public static final double kAKickerVoltSecondsSquaredPerRotation = 0.0019767;

    public static final double kFShooterOutputUnit100MsPerSensorUnit =
        kVShooterVoltSecondsPerRotation
            * kFalconOutputUnitsPerVolt
            / kShooterGearRatio
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;
    public static final double kFKickerOutputUnit100MsPerSensorUnit =
        kVKickerVoltSecondsPerRotation
            / kKickerGearRatio
            * kFalconOutputUnitsPerVolt
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kPShooterVoltSecondsPerRotation = 0.01; // 0.5
    public static final double kPShooterOutputUnit100MsPerSensorUnit =
        kPShooterVoltSecondsPerRotation
            * kFalconOutputUnitsPerVolt
            / kShooterGearRatio
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kPKickerVoltSecondsPerRotation = 0.1; // 0.5
    public static final double kPKickerOutputUnit100MsPerSensorUnit =
        kPKickerVoltSecondsPerRotation
            / kKickerGearRatio
            * kFalconOutputUnitsPerVolt
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kShooterControlVelocityToleranceRotationsPerSecond = 1;
    public static final double kShooterControlVelocityToleranceSensorUnitsPer100Ms =
        kShooterControlVelocityToleranceRotationsPerSecond
            * kSecondsPer100Ms
            * kFalconSensorUnitsPerRotation;

    public static final double kKickerControlVelocityToleranceRotationsPerSecond =
        kShooterControlVelocityToleranceRotationsPerSecond
            * (kKickerFreeSpeedRotationsPerSecond / kShooterFreeSpeedRotationsPerSecond);
    public static final double kKickerControlVelocityToleranceSensorUnitsPer100Ms =
        kKickerControlVelocityToleranceRotationsPerSecond
            * kSecondsPer100Ms
            * kFalconSensorUnitsPerRotation;

    public static final double kShooterFeedforwardScale = 0.86;
    public static final double kKickerFeedforwardScale = 0.88;

    public static final double kShooterFeedVelocityTolerance = 3;

    public static final double kDefaultTangentialVelocityRatio =
        1.0 / 3.0; // kicker tangential velocity / shooter tangential velocity
  }

  public static final class ClimbElevatorConstants {

    public static final double kElevatorControlLoopTimeSeconds = 0.01;
    // SLIDE CONSTANTS
    public static final int kElevatorTalonPort = 40;
    public static final double kElevatorSupplyCurrentLimitAmps = 25;
    public static final double kElevatorSupplyCurrentThresholdAmps = 30;
    public static final double kElevatorSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kElevatorStatorCurrentLimitAmps = 40;
    public static final double kElevatorStatorCurrentThresholdAmps = 45;
    public static final double kElevatorStatorCurrentThresholdTimeSecs = 0.05;

    public static final int kElevatorSlotSensorTopPort = 0; // dio
    public static final int kElevatorSlotSensorBottomPort = 1; // dio

    public static final double kElevatorTopStopPosition = Units.inchesToMeters(0);
    public static final double kElevatorBottomStopPosition = Units.inchesToMeters(-21.25);

    public static final double kElevatorSlotSensorTopPosition = Units.inchesToMeters(20.75);
    public static final double kElevatorSlotSensorBottomPosition = Units.inchesToMeters(0.25);
    public static final double kSlotSensorDebounceTime = 0.1;

    public static final int kElevatorServoPort = 0; // pwm
    public static final double kElevatorServoRange = 270;
    public static final double kElevatorServoEngageValue = 0;
    public static final double kElevatorServoDisengageValue = 0.5;

    public static final double kSprocketCircumferenceMeters = 0.0323342 * Math.PI;

    public static final double kElevatorGearRatio = 32.67; // motor turns/pulley turns
    public static final double kElevatorDistancePerMotorRevMeters =
        kSprocketCircumferenceMeters / kElevatorGearRatio;

    public static final double kElevatorFreeSpeedMetersPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kElevatorGearRatio * kSprocketCircumferenceMeters;

    public static final double kSlideDistancePerPulseMeters =
        kElevatorDistancePerMotorRevMeters / kFalconSensorUnitsPerRotation;

    // Slide characterization constants: UNLOADED (not carrying robot)
    public static final double ksElevatorUnloadedVolts = 0.70015;
    public static final double kgElevatorUnloadedVolts = 0.010378;
    public static final double kvElevatorUnloadedVoltSecondsPerMeter = 30.0;
    public static final double kaElevatorUnloadedVoltSecondsSquaredPerMeter = 0.01;

    // Slide characterization constants: LOADED ( carrying robot)
    public static final double ksElevatorLoadedVolts = 0.44256;
    public static final double kgElevatorLoadedVolts = -0.50; // this is negative because gravity fights the downward motion when loaded
    // --
    // retracting the elevator moves the robot up.
    public static final double kvElevatorLoadedVoltSecondsPerMeter = 30.0;
    public static final double kaElevatorLoadedVoltSecondsSquaredPerMeter = 0.18; // these are recalc gains -- the ka from sysid was lost in the noise

    public static final double kPElevatorUnloadedVoltsPerMeter = 100;
    public static final double kDElevatorUnloadedVoltSecondsPerMeter = 0.05;

    public static final double kPElevatorLoadedVoltsPerMeter = 280;
    public static final double kDElevatorLoadedVoltSecondsPerMeter = 10;

    public static final double kElevatorPositionToleranceMeters = 0.008;
    public static final double kElevatorVelocityToleranceMetersPerSecond = 0.01;

    public static final double kElevatorMaxSpeedMetersPerSecond =
        kElevatorFreeSpeedMetersPerSecond * 0.95;
    // Find maximum simultaneously achievable acceleration
    public static final double kElevatorMaxAccelerationMetersPerSecondSquaredUnloaded = 0.8;

    public static final double kSlideMaxAccelerationMetersPerSecondSquaredLoaded = 0.5;

    // Constraint for the motion profilied elevator controller (unloaded mode)
    public static final TrapezoidProfile.Constraints kElevatorControllerConstraintsUnloaded =
        new TrapezoidProfile.Constraints(
            kElevatorMaxSpeedMetersPerSecond,
            kElevatorMaxAccelerationMetersPerSecondSquaredUnloaded);

    // Constraint for the motion profilied elevator controller (loaded mode)
    public static final TrapezoidProfile.Constraints kElevatorControllerConstraintsLoaded =
        new TrapezoidProfile.Constraints(
            kElevatorMaxSpeedMetersPerSecond, kSlideMaxAccelerationMetersPerSecondSquaredLoaded);
  }

  public static final class ClimbArmConstants {
    public static final double kArmControlLoopTimeSeconds = 0.01;

    public static final double kArmGearingChange = 8.0 / 11.0;
    // ARM CONSTANTS
    public static final int kArmTalonPort = 41;
    public static final double kArmSupplyCurrentLimitAmps = 25;
    public static final double kArmSupplyCurrentThresholdAmps = 30;
    public static final double kArmSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kArmStatorCurrentLimitAmps = 25;
    public static final double kArmStatorCurrentThresholdAmps = 30;
    public static final double kArmStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kArmStatorCurrentSpikeThresholdAmps = 30;
    public static final double kArmStatorCurrentSpikeDebounceTimeSeconds = 0.2;

    public static final double kArmGearRatio = 57.6 * (16.0 / 22.0); // motor turns/pinion turns
    public static final double kArmRotationsPerMotorRev = 1 / kArmGearRatio;
    public static final double kArmRotationsPerPulse =
        kArmRotationsPerMotorRev / kFalconSensorUnitsPerRotation;

    public static final double kSprocketCircumferenceMeters = 0.0323342 * Math.PI;

    // arm characteristics used in angular mode
    public static final double kArmMomentOfIntertia = 5;
    public static final double kArmLengthMeters = 2; // length to center of mass
    public static final double kGravityMetersPerSecondSquared = 9.81;

    // arm characteristics used in translation mode
    public static final double kArmDistancePerMotorRevMeters =
        kSprocketCircumferenceMeters / kArmGearRatio;
    // public static final double kArmRotationToleranceRadians = 0.05;

    public static final double kArmFreeSpeedMetersPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kArmGearRatio * kSprocketCircumferenceMeters;

    public static final double kArmRotationTolerance = 0.01;
    // ARM TRANSLATION CONSTANTS
    public static final double ksArmTranslationVolts = 0.55953;
    public static final double kgArmTranslationVolts = 0.18092;
    public static final double kvArmTranslationVoltSecondsPerMeter = 62.80;
    public static final double kaArmTranslationVoltSecondsSquaredPerMeter = 0.04;

    public static final double kArmMaxSpeedTranslationMetersPerSecond =
        kArmFreeSpeedMetersPerSecond * 0.9 / kArmGearingChange;
    public static final double kArmMaxAccelerationTranslationMetersPerSecondSquared =
        10 * kArmGearingChange;

    // Constraint for the motion profilied arm rotation controller
    public static final TrapezoidProfile.Constraints kArmControllerConstraintsTranslation =
        new TrapezoidProfile.Constraints(
            kArmMaxSpeedTranslationMetersPerSecond,
            kArmMaxAccelerationTranslationMetersPerSecondSquared);

    public static final double kPArmTranslationVoltsPerMeter = 400 * kArmGearingChange;
    public static final double kDArmTranslationVoltSecondsPerMeter = 0.275 * kArmGearingChange;
    public static final double kIArmTranslationVoltsPerMeter = 30;
    public static final double kArmIntegratorMaxVolts = 0.5;
    public static final double ksArmUnloadedVolts = 0.47;
    public static final double kgArmUnloadedVolts = 0.02;
    public static final double kvArmUnloadedVoltSecondsPerMeter = 63.445 * kArmGearingChange;
    public static final double kaArmUnloadedVoltSecondsSquaredPerMeter = 0.01 * kArmGearingChange;

    public static final double kArmTranslationToleranceMeters = 0.02;
    public static final double kArmTranslationVelocityToleranceMetersPerSecond =
        0.05 / kArmGearingChange;
  }

  public static final class Swerve {
    public static final int pigeonID = 9;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.508;
    public static final double wheelBase = 0.508;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    
    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    // public static final double angleKP = chosenModule.angleKP;
    // public static final double angleKI = chosenModule.angleKI;
    // public static final double angleKD = chosenModule.angleKD;
    // public static final double angleKF = chosenModule.angleKF;

    public static final double angleKP = 60;
    public static final double angleKI = 0;
    public static final double angleKD = .05;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 2.5;
    public static final double driveKI = 3;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 3; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.3662109375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.116455078125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.1572265625);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }
    /* Back Left Module - Module 3- */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.18701171875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

  }
}

