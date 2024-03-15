// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.Intake;
import frc.robot.subsystems.NoteHandling.Pivot;
import frc.robot.subsystems.NoteHandling.Shooter;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandBuilder {
  /** Example static factory for an autonomous command. */
  public static final Command Fire(Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    Command command = NoteHandlingCommandBuilder.autoShooterSpeaker(pivot, shooter, feeder, indexer)
                      .alongWith(new DriveWithController(()->0, ()->0, ()->0, ()->true, ()->true, poseEstimation::getAngleToSpeaker, ()-> false, poseEstimation::getAngleToAmp, ()->false, ()-> false, ()-> 0, drivetrain, poseEstimation))
                      .until(()->feeder.getCurrentState() == GeneralRollerStates.StateForwardFast)
                      .andThen(new WaitCommand(.5))
                      .andThen(NoteHandlingCommandBuilder.autoShooterOff(pivot, shooter, feeder, indexer, intake));
    return command;
  }

  public static final Command twoNoteAt1(Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    Command command = new SequentialCommandGroup(
      Fire(drivetrain, poseEstimation, intake, shooter, pivot, feeder, indexer),
      // NoteHandlingCommandBuilder.intake(intake, indexer, feeder),
      driveToNote(drivetrain, poseEstimation, 0, -50),
      Fire(drivetrain, poseEstimation, intake, shooter, pivot, feeder, indexer),
      // NoteHandlingCommandBuilder.intake(intake, indexer, feeder),
      driveToNote(drivetrain, poseEstimation, 1, 140),
      Fire(drivetrain, poseEstimation, intake, shooter, pivot, feeder, indexer)
      //NoteHandlingCommandBuilder.intakeOff(intake, indexer, feeder, pivot)

      );
    return command;
  }

  public static final Command driveToNote(Drivetrain drivetrain, PoseEstimation poseEstimation, int index, double degrees) {
      Pose2d pos = poseEstimation.getFieldConstants().getRingPositions()[index];
      Rotation2d rotation = Rotation2d.fromDegrees(degrees);
      Command command = new SequentialCommandGroup(
      DriveCommandBuilder.driveToPosition(drivetrain, poseEstimation, ()->new Pose2d(pos.getTranslation(), rotation)).until(()->poseEstimation.dist(poseEstimation.getCurrentPose(), pos)<.25),
      new InstantCommand(()->drivetrain.drive(new Translation2d(0, 0), 0, false, false)));
    return command;
  } 
}
