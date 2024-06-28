package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb.Climber;
import frc.robot.subsystems.Climb.Climber.ClimberStates;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystems.NoteHandling.Pivot;
import frc.robot.subsystems.NoteHandling.Pivot.PivotStates;
import frc.robot.subsystems.NoteHandling.Shooter;
import frc.robot.subsystems.NoteHandling.Shooter.ShooterStates;

public class ClimbCommandBuilder {
        // public static Command enableClimb(Climber climber) {
        //     Command command = new InstantCommand(()->climber.requestState(ClimberStates.StatePrimed), climber);
        //     return command;
        // }

        public static Command upPosition(Climber climber) {
            //if (climber.getRequestedState() == ClimberStates.StateNotPrimed) return new InstantCommand(()->climber.requestState(ClimberStates.StateNotPrimed)); 
            Command command = new InstantCommand(()->climber.requestState(ClimberStates.StateUp), climber);
            return command;
        }

        public static Command climb(Climber climber) {
            Command command = new SequentialCommandGroup(
                            new InstantCommand(()->climber.requestState(ClimberStates.StateDownPosition), climber)
            );   
            return command;
        }
}
