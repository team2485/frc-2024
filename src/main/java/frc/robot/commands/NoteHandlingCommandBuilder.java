package frc.robot.commands;

// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.Intake;
import frc.robot.subsystems.NoteHandling.Pivot;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystems.NoteHandling.Intake.IntakeStates;
import frc.robot.subsystems.NoteHandling.Pivot.PivotStates;
import frc.robot.subsystems.NoteHandling.Shooter;
import frc.robot.subsystems.NoteHandling.Shooter.ShooterStates;

public class NoteHandlingCommandBuilder {
    public static Command intake(Intake intake, GeneralRoller indexer, GeneralRoller feeder) {
        Command command = new ParallelCommandGroup(
                                new InstantCommand(()->intake.requestState(IntakeStates.StateIntake), intake),
                                new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateForward), indexer),
                                new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateReverse), feeder)
                                );
        return command;
    }

    public static Command outtake(Intake intake, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                                new InstantCommand(()->intake.requestState(IntakeStates.StateOuttake), intake),
                                new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateReverse), indexer)
                                );
        return command;
    }
    
    public static Command intakeOff(Intake intake, GeneralRoller indexer, GeneralRoller feeder) {
        Command command = new ParallelCommandGroup(
                                new RunCommand(()->intake.requestState(IntakeStates.StateOff), intake),
                                new RunCommand(()->indexer.requestState(GeneralRollerStates.StateOff), indexer),
                                new RunCommand(()->feeder.requestState(GeneralRollerStates.StateOff), feeder)
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


    public static Command feederOff(GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new ParallelCommandGroup(
                        new InstantCommand(()->feeder.requestState(GeneralRollerStates.StateOff), feeder),
                        new InstantCommand(()->indexer.requestState(GeneralRollerStates.StateOff), indexer)                 
                        );
        return command;
    }

    public static Command shooterSpeaker(Shooter shooter){
        Command command = new InstantCommand(()->shooter.requestState(ShooterStates.StateSpeaker), shooter);
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

    public static Command autoShooterOff(Pivot pivot, Shooter shooter, GeneralRoller feeder, GeneralRoller indexer, Intake intake) {
        Command command = new ParallelCommandGroup(
                        new InstantCommand(()->pivot.requestState(PivotStates.StateDown)),
                        new InstantCommand(()->intake.requestState(IntakeStates.StateOff)),
                        shooterOff(shooter, feeder, indexer)
                        );
        return command;
    }

    public static Command shoot(Shooter shooter, GeneralRoller feeder, GeneralRoller indexer) {
        Command command = new SequentialCommandGroup(
                        new RunCommand(()->shooter.requestState(ShooterStates.StateSpeaker)).until(()->shooter.getCurrentState() == ShooterStates.StateSpeaker),
                        runFeeder(feeder, indexer)
                        );

        return command;
    }
}
