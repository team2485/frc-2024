// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.sound.sampled.BooleanControl;
import javax.xml.crypto.dsig.spec.XSLTTransformParameterSpec;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DriveWithController extends Command {
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotSpeedSupplier;
  private final BooleanSupplier m_fieldRelative;
  private final BooleanSupplier m_aimingAtSpeaker;
  private final DoubleSupplier m_speakerAngle;
  private final BooleanSupplier m_aimingAtAmp;
  private final BooleanSupplier m_aimingAtNote;
  private final BooleanSupplier m_noteExists;
  private final DoubleSupplier m_angleToNote;
  private final DoubleSupplier m_angleToNoteSetpoint;
  private final DoubleSupplier m_ampAngle;
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_passAngle;
  private final BooleanSupplier m_passing;
  private final PIDController rotationOverrideController = new PIDController(.1*.9, 0.008, .015*.75);
  //.175, 0, .01
  private final PIDController xOverrideController = new PIDController(5, 0, 0);
  private final PoseEstimation mPoseEstimation;

  public DriveWithController(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSpeedSupplier,
      BooleanSupplier fieldRelative,
      BooleanSupplier aimingAtSpeaker,
      DoubleSupplier speakerAngle,
      BooleanSupplier aimingAtAmp,
      DoubleSupplier ampAngle,
      BooleanSupplier passing,
      DoubleSupplier passAngle,
      BooleanSupplier aimingAtNote,
      BooleanSupplier noteExists,
      DoubleSupplier angleToNote,
      DoubleSupplier angleToNoteSetpoint,
      Drivetrain drivetrain,
      PoseEstimation poseEstimation) {

    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;
    this.m_rotSpeedSupplier = rotSpeedSupplier;
    this.m_fieldRelative = fieldRelative;
    this.m_aimingAtSpeaker = aimingAtSpeaker;
    this.m_speakerAngle = speakerAngle;
    this.m_aimingAtAmp = aimingAtAmp;
    this.m_ampAngle = ampAngle;
    this.m_aimingAtNote = aimingAtNote;
    this.m_noteExists = noteExists;
    this.m_angleToNote = angleToNote;
    this.m_angleToNoteSetpoint = angleToNoteSetpoint;
    this.m_passAngle = passAngle;
    this.m_passing = passing;

    this.mPoseEstimation = poseEstimation;
    this.m_drivetrain = drivetrain;

    rotationOverrideController.enableContinuousInput(-180, 180);
    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("xbox right x", m_xSpeedSupplier.getAsDouble());
    // SmartDashboard.putNumber("xbox left x", m_ySpeedSupplier.getAsDouble());
    // SmartDashboard.putNumber("xbox left y", m_rotSpeedSupplier.getAsDouble());

    final int xSign = (int)(Math.abs(m_xSpeedSupplier.getAsDouble())/m_xSpeedSupplier.getAsDouble());
    double xSpeed =
        map(-MathUtil.applyDeadband(Math.abs(m_xSpeedSupplier.getAsDouble()*m_xSpeedSupplier.getAsDouble()), kDriverLeftYDeadband)
            * kTeleopMaxSpeedMetersPerSecond, 0, 1, 0, Swerve.maxSpeed) * -xSign;

    final int ySign = (int)(Math.abs(m_ySpeedSupplier.getAsDouble())/m_ySpeedSupplier.getAsDouble());
    double ySpeed =
        map(-MathUtil.applyDeadband(Math.abs(m_ySpeedSupplier.getAsDouble()*m_ySpeedSupplier.getAsDouble()), kDriverLeftXDeadband)
          * kTeleopMaxSpeedMetersPerSecond, 0, 1, 0, Swerve.maxSpeed) * ySign;


    final int rotSign = (int)(Math.abs(m_rotSpeedSupplier.getAsDouble())/m_rotSpeedSupplier.getAsDouble());
    double rot = Math.abs(map(-MathUtil.applyDeadband(m_rotSpeedSupplier.getAsDouble()*m_rotSpeedSupplier.getAsDouble(), kDriverRightXDeadband)
            * kTeleopMaxAngularSpeedRadiansPerSecond, 0, 1, 0, Swerve.maxAngularVelocity)) * rotSign;

    boolean fieldRelative = m_fieldRelative.getAsBoolean();

    final boolean aimingAtSpeaker = m_aimingAtSpeaker.getAsBoolean();

    final double speakerAngle = m_speakerAngle.getAsDouble();

    final boolean aimingAtAmp = m_aimingAtAmp.getAsBoolean();

    final boolean passing = m_passing.getAsBoolean();

    final double passAngle = m_passAngle.getAsDouble();

    final double ampAngle = m_ampAngle.getAsDouble();

    final boolean aimingAtNote = m_aimingAtNote.getAsBoolean();

    final boolean noteExists = m_noteExists.getAsBoolean();

    final double angleToNote = m_angleToNote.getAsDouble();

    final double angleToNoteSetpoint = m_angleToNoteSetpoint.getAsDouble();

    if (aimingAtSpeaker) {
      xSpeed*=.1;
      ySpeed*=.1;
      // double targetAngle = speakerAngle;
      // double distance = mPoseEstimation.getDistanceToSpeaker();
      // targetAngle+= ySpeed*5*distance; 
      rot = -rotationOverrideController.calculate(m_drivetrain.getYawMod().getDegrees(), speakerAngle);
      SmartDashboard.putNumber("AngleError", -rotationOverrideController.getPositionError());
    }

    if (aimingAtAmp) {
      rot = -rotationOverrideController.calculate(m_drivetrain.getYawMod().getDegrees(), 90);

      // xSpeed = xOverrideController.calculate(mPoseEstimation.getCurrentPose().getX(), mPoseEstimation.getFieldConstants().getAmpPos().getX());
      // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      //   xSpeed *= -1;
      // ySpeed *= .4;
    
    }

    if (passing) {
      //rot = -rotationOverrideController.calculate(m_drivetrain.getYawMod().getDegrees(), passAngle);
    }

    if (aimingAtNote) {
      if (noteExists) {
        rot = rotationOverrideController.calculate(angleToNote, angleToNoteSetpoint);
      }
      fieldRelative = false;
    }

    Translation2d centerOfRotation = aimingAtSpeaker ? new Translation2d(-kPivotToRobot, 0) : new Translation2d();

    m_drivetrain.drive(new Translation2d(xSpeed, ySpeed), rot, fieldRelative, true, new Translation2d());

    // System.out.println(m_driver.getRightTriggerAxis());
  }

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, false, false, new Translation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
