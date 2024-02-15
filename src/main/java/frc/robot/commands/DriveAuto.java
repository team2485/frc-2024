// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DriveAuto extends Command {
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotSpeedSupplier;
  //private final BooleanSupplier m_fieldRelative;

  private final Drivetrain m_drivetrain;


  public DriveAuto(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSpeedSupplier,
    //  BooleanSupplier fieldRelative,
      Drivetrain drivetrain) {

    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;
    this.m_rotSpeedSupplier = rotSpeedSupplier;
   // this.m_fieldRelative = fieldRelative;

    this.m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.driveAuto(new ChassisSpeeds(-m_xSpeedSupplier.getAsDouble(), m_ySpeedSupplier.getAsDouble(), m_rotSpeedSupplier.getAsDouble()));
    
    // System.out.println(m_driver.getRightTriggerAxis());
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
