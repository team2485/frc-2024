package frc.robot.commands;

// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.NoteHandling.Intake;
import frc.robot.subsystems.NoteHandling.Intake.IntakeStates;

public class NoteHandlingCommandBuilder {
    public static Command intake(Intake intake) {
        Command command = new InstantCommand(()->intake.requestState(IntakeStates.StateIntake), intake);
        return command;
    }

    public static Command outtake(Intake intake) {
        Command command = new InstantCommand(()->intake.requestState(IntakeStates.StateOuttake), intake);
        return command;
    }
    
    public static Command intakeOff(Intake intake) {
        Command command = new InstantCommand(()->intake.requestState(IntakeStates.StateOff), intake);
        return command;
    }
}
