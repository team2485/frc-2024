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

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandBuilder {
  /** Example static factory for an autonomous command. */

  public static final Command Fire(Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    Command command = NoteHandlingCommandBuilder.autoShooterSpeaker(pivot, shooter, feeder, indexer)
                      .alongWith(new DriveWithController(()->0, ()->0, ()->0, ()->true, ()->true, poseEstimation::getAngleToSpeaker, ()-> false, poseEstimation::getAngleToAmp, ()->false, poseEstimation::getAngleToStage, ()->false, ()-> false, ()-> 0, ()->0, drivetrain, poseEstimation))
                      .until(()->feeder.getCurrentState() == GeneralRollerStates.StateForwardFast)
                      .andThen(new WaitCommand(.5))
                      .andThen(NoteHandlingCommandBuilder.autoShooterOffish(pivot, shooter, feeder, indexer, intake));
    return command;
  }

  public static final Command FireShooterOff(Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    Command command = NoteHandlingCommandBuilder.autoShooterSpeaker(pivot, shooter, feeder, indexer)
                      .alongWith(new DriveWithController(()->0, ()->0, ()->0, ()->true, ()->true, poseEstimation::getAngleToSpeaker, ()-> false, poseEstimation::getAngleToAmp, ()->false, poseEstimation::getAngleToStage, ()->false, ()-> false, ()-> 0, ()->0, drivetrain, poseEstimation))
                      .until(()->feeder.getCurrentState() == GeneralRollerStates.StateForwardFast)
                      .andThen(new WaitCommand(.5))
                      .andThen(NoteHandlingCommandBuilder.autoShooterOff(pivot, shooter, feeder, indexer, intake));
    return command;
  }

  public static final Command choreoTestAuto(Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    return NoteHandlingCommandBuilder.autoSetGyro(drivetrain, poseEstimation).andThen(Choreo.choreoSwerveCommand(Choreo.getTrajectory("TestPath1"), 
        poseEstimation::getCurrentVisionlessPose, 
        Choreo.choreoSwerveController(new PIDController(2, 0, 0), new PIDController(2, 0, 0), new PIDController(0, 0, 0)), 
        drivetrain::driveAuto, 
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red, 
        drivetrain));
  }

  public static final Command runNoteScoringChoreoAuto(String autoName, Drivetrain drivetrain, PoseEstimation poseEstimation, Intake intake, Shooter shooter, Pivot pivot, GeneralRoller feeder, GeneralRoller indexer) {
    ArrayList<ChoreoTrajectory> pathList = Choreo.getTrajectoryGroup(autoName);
    SequentialCommandGroup commandsList = new SequentialCommandGroup();

    // make auto

    commandsList.addCommands(NoteHandlingCommandBuilder.autoSetGyro(drivetrain, poseEstimation));

    commandsList.addCommands(Fire(drivetrain, poseEstimation, intake, shooter, pivot, feeder, indexer));

    commandsList.addCommands(NoteHandlingCommandBuilder.intakeAuto(intake, indexer, feeder));

    for (int i = 0; i < pathList.size(); i++) {
      commandsList.addCommands(       
        Choreo.choreoSwerveCommand(pathList.get(0), 
        poseEstimation::getCurrentPose, 
        Choreo.choreoSwerveController(new PIDController(2, 0, 0), new PIDController(2, 0, 0), new PIDController(0, 0, 0)), 
        drivetrain::driveAuto, 
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red, 
        drivetrain)
      );

      commandsList.addCommands(Fire(drivetrain, poseEstimation, intake, shooter, pivot, feeder, indexer));

      commandsList.addCommands(NoteHandlingCommandBuilder.intakeAuto(intake, indexer, feeder));
    }

    commandsList.addCommands(NoteHandlingCommandBuilder.intakeOff(intake, indexer, feeder, pivot, null));

    return commandsList;
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
      new InstantCommand(()->drivetrain.drive(new Translation2d(0, 0), 0, false, false, new Translation2d())));
    return command;
  } 
}
