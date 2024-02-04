// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.NoteHandlingCommandBuilder;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.Intake;
import frc.robot.subsystems.NoteHandling.Pivot;
import frc.robot.subsystems.NoteHandling.Shooter;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.GeneralRollerConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final PoseEstimation m_poseEstimation = new PoseEstimation(m_drivetrain::getYaw, m_drivetrain::getModulePositionsInverted);
  private final Intake m_intake = new Intake();
  private final GeneralRoller m_indexer = new GeneralRoller(kIndexerPort, true);
  private final GeneralRoller m_feeder = new GeneralRoller(kFeederPort, true);
  private final Shooter m_shooter = new Shooter();

  //private final GeneralRoller m_feeder = new GeneralRoller(kFeederPort, false);
  private final Pivot m_pivot = new Pivot();
  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(kDriverPort);
  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(kOperatorPort);


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_poseEstimation.addDashboardWidgets(Shuffleboard.getTab("Swerve"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      new DriveWithController(
          m_driver::getLeftY,
          m_driver::getLeftX,
          m_driver::getRightX,
          () -> !m_driver.rightBumper().getAsBoolean(),
          m_drivetrain));

    m_driver.x().onTrue(new InstantCommand(m_drivetrain::zeroGyro)
                .alongWith(new InstantCommand(m_drivetrain::resetToAbsolute)));

    m_driver.rightTrigger().onTrue(NoteHandlingCommandBuilder.intake(m_intake, m_indexer))
                           .onFalse(NoteHandlingCommandBuilder.intakeOff(m_intake, m_indexer));

    m_driver.leftBumper().onTrue(NoteHandlingCommandBuilder.outtake(m_intake, m_indexer))
                         .onFalse(NoteHandlingCommandBuilder.intakeOff(m_intake, m_indexer));

    m_operator.upperPOV().onTrue(NoteHandlingCommandBuilder.pivotToAmp(m_pivot));

    m_operator.lowerPOV().onTrue(NoteHandlingCommandBuilder.pivotDown(m_pivot));

    // m_operator.rightBumper().onTrue(NoteHandlingCommandBuilder.runFeeder(m_feeder))
    //                         .onFalse(NoteHandlingCommandBuilder.feederOff(m_feeder));
    // m_operator.rightTrigger().whileTrue(NoteHandlingCommandBuilder.shooterSpeaker(m_shooter))
    //                          .whileFalse(NoteHandlingCommandBuilder.shooterCoast(m_shooter));
    m_operator.rightTrigger().whileTrue(NoteHandlingCommandBuilder.shoot(m_shooter, m_feeder, m_indexer))
                            .whileFalse(NoteHandlingCommandBuilder.shooterCoast(m_shooter, m_feeder, m_indexer));

    m_operator.rightBumper().onTrue(NoteHandlingCommandBuilder.runFeeder(m_feeder, m_indexer))
                            .onFalse(NoteHandlingCommandBuilder.feederOff(m_feeder, m_indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
