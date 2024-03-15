package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kDriveTolerance;
import static frc.robot.Constants.DriveConstants.kRotationTolerance;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.Intake;
import frc.robot.subsystems.NoteHandling.Pivot;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystems.NoteHandling.Intake.IntakeStates;
import frc.robot.subsystems.NoteHandling.Pivot.PivotStates;
import frc.robot.subsystems.NoteHandling.Shooter;
import frc.robot.subsystems.NoteHandling.Shooter.ShooterStates;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;

public class NoteHandlingCommandBuilder {

    public static Command intake(Intake intake, GeneralRoller indexer, GeneralRoller feeder, WL_CommandXboxController driver) {
        Command command = new ParallelCommandGroup(
                                new InstantCommand(()->intake.requestState(IntakeStates.StateIntake), intake),
                                new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateForward), indexer),
                                new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateReverse), feeder),
                                new WaitCommand(.1).andThen(new WaitUntilCommand(()->feeder.getCurrent() > 23)).andThen(new StartEndCommand(()->driver.setRumble(RumbleType.kRightRumble, 0.75), ()->driver.setRumble(RumbleType.kRightRumble, 0)))
                                );
        return command;
    }

    public static Command intakeAuto(Intake intake, GeneralRoller indexer, GeneralRoller feeder) {
        Command command = new ParallelCommandGroup(
                                new InstantCommand(()->intake.requestState(IntakeStates.StateIntake), intake),
                                new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateForward), indexer),
                                new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateReverse), feeder)
                                );
        return command;
    }


    // public static Command outtake(Intake intake, GeneralRoller indexer) {
    //     Command command = new ParallelCommandGroup(
    //                             new InstantCommand(()->intake.requestState(IntakeStates.StateOuttake), intake),
    //                             new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateReverse), indexer)
    //                             );
    //     return command;
    // }

    public static Command outtake(Intake intake, GeneralRoller indexer, GeneralRoller feeder, Pivot pivot) {
        Command command = new SequentialCommandGroup(
                                new RunCommand(()->pivot.requestState(PivotStates.StateOuttake)).until(()->pivot.getCurrentState() == PivotStates.StateOuttake),
                                new ParallelCommandGroup(
                                    new InstantCommand(()->intake.requestState(IntakeStates.StateOuttake), intake),
                                    new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateReverse), indexer),
                                    new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateReverse), feeder)
                                )
                                );
        return command;
    }
    
    public static Command intakeOff(Intake intake, GeneralRoller indexer, GeneralRoller feeder, Pivot pivot, WL_CommandXboxController driver) {
        Command command = new ParallelCommandGroup(
                                new InstantCommand(()->intake.requestState(IntakeStates.StateOff), intake),
                                new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateOff), indexer),
                                new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateOff), feeder),
                                new InstantCommand(()->pivot.requestState(PivotStates.StateDown), pivot)
                                );
        return command;
    }

    public static Command pivotToAmp(Pivot pivot) {
        Command command = new InstantCommand(()->pivot.requestState(PivotStates.StateAmp), pivot); 
        return command;
    }   

    public static Command pivotDown(Pivot pivot) {
        Command command = new InstantCommand(()->pivot.requestState(PivotStates.StateDown), pivot);
        return command;
    }

    public static Command runFeeder(GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                                new RunCommand(()->feeder.requestState(GeneralRollerStates.StateForwardFast), feeder),
                                new RunCommand(()->indexer.requestState(GeneralRollerStates.StateForwardFast), indexer)                 
                                );
        return command;
    }

    public static Command runFeederIsh(GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                                new RunCommand(()->feeder.requestState(GeneralRollerStates.StateForward), feeder),
                                new RunCommand(()->indexer.requestState(GeneralRollerStates.StateForwardFast), indexer)                 
                                );
        return command;
    }


    public static Command feederOff(GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                        new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateOff), feeder),
                        new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateOff), indexer)                 
                        );
        return command;
    }

    public static Command shooterSpeaker(Shooter shooter, GeneralRoller feeder, GeneralRoller indexer){
        Command command = new SequentialCommandGroup(
            new RunCommand(()->shooter.requestState(ShooterStates.StateSpeaker), shooter).until(()->shooter.getCurrentState()==ShooterStates.StateSpeaker),
            runFeeder(feeder, indexer)
            );

        return command;
    }

    public static Command shooterOff(Shooter shooter, GeneralRoller feeder, GeneralRoller indexer){
        Command command = new ParallelCommandGroup(
            new InstantCommand(()->shooter.requestState(ShooterStates.StateOff), shooter),
            feederOff(feeder, indexer));
        return command;
    }

    public static Command autoShooterSpeaker(Pivot pivot, Shooter shooter, GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                        new RunCommand(()->pivot.requestState(PivotStates.StateShooter), pivot), 
                        new RunCommand(()->shooter.requestState(ShooterStates.StateSpeaker), shooter),
                        new RunCommand(()->feeder.requestState(GeneralRollerStates.StateOff), feeder),
                        new RunCommand(()->indexer.requestState(GeneralRollerStates.StateOff), indexer)
                        ).until(()->pivot.getCurrentState() == PivotStates.StateShooter && shooter.getCurrentState() == ShooterStates.StateSpeaker)
                        .andThen(
                            new ParallelCommandGroup(
                                new RunCommand(()->feeder.requestState(GeneralRollerStates.StateForwardFast), feeder),
                                new RunCommand(()->indexer.requestState(GeneralRollerStates.StateForwardFast), indexer)
                            )
                        );
        return command.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command autoSetGyro(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation) {
        // if (angle > 90) angle = 180 - angle;
        // if (angle < -90) angle = -180 - angle; 
        return new SequentialCommandGroup(new WaitCommand(.015), new InstantCommand(()-> m_drivetrain.setCustomYaw(m_poseEstimation.getAngleFromTags())));
    }

    public static Command autoShooterOff(Pivot pivot, Shooter shooter, GeneralRoller feeder, GeneralRoller indexer, Intake intake) {
        Command command = new ParallelCommandGroup(
                        new InstantCommand(()->pivot.requestState(PivotStates.StateDown), pivot),
                        new InstantCommand(()->intake.requestState(IntakeStates.StateOff), intake),
                        shooterOff(shooter, feeder, indexer)
                        );
        return command;
    }



    public static Command shoot(Shooter shooter, GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new SequentialCommandGroup(
                        new RunCommand(()->shooter.requestState(ShooterStates.StateSpeaker), shooter).until(()->shooter.getCurrentState() == ShooterStates.StateSpeaker),
                        runFeeder(feeder, indexer)
                        );

        return command;
    }

    public static Command shootTrap(Shooter shooter, GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new SequentialCommandGroup(
                        new RunCommand(()->shooter.requestState(ShooterStates.StateTrap), shooter).until(()->shooter.getCurrentState() == ShooterStates.StateSpeaker),
                        runFeeder(feeder, indexer)
                        );

        return command;
    }

    public static Command autoAmp(Drivetrain drivetrain, Pivot pivot, Shooter shooter, GeneralRoller feeder, GeneralRoller indexer, PoseEstimation poseEstimation) {
        Command command = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunCommand(()->pivot.requestState(PivotStates.StateAmp), pivot),
                new RunCommand(()->shooter.requestState(ShooterStates.StateCoast), shooter)
            ).until(()->pivot.getCurrentState() == PivotStates.StateAmp),
            runFeederIsh(feeder, indexer)
        );
        return command;
    }
    

    // public static Command autoAmp(Drivetrain drivetrain, Pivot pivot, Shooter shooter, GeneralRoller feeder, GeneralRoller indexer, PoseEstimation poseEstimation, Supplier<Pose2d> ampPos) {
    //     Command command = new SequentialCommandGroup(
    //         DriveCommandBuilder.driveToPosition(drivetrain, poseEstimation, ampPos)
    //             .until(()->poseEstimation.dist(poseEstimation.getCurrentPose(), ampPos.get()) < kDriveTolerance
    //                        && Math.abs(drivetrain.getYawAbsolute().getDegrees()-ampPos.get().getRotation().getDegrees()) < kRotationTolerance),
    //         new ParallelCommandGroup(
    //             new RunCommand(()->pivot.requestState(PivotStates.StateAmp), pivot),
    //             new RunCommand(()->shooter.requestState(ShooterStates.StateCoast), shooter)
    //         ).until(()->pivot.getCurrentState() == PivotStates.StateAmp),
    //         runFeeder(feeder, indexer)
    //     );
    //     return command;
    // }
}
