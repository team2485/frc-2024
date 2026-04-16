package frc.robot.subsystems.NoteHandling;
// import edu.wpi.first.math.controller.PIDController;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum IntakeStates {
    StateOff,
    StateIntake,
    StateOuttake,
  }

  public static IntakeStates m_subsystemNameCurrentState;
  public static IntakeStates m_subsystemNameRequestedState;

  // You may need more than one motor
  private final SparkMax m_spark = new SparkMax(kIntakePort, MotorType.kBrushless);
  // Units depend on the units of the setpoint() and calculate() methods. This example will use meters
  private double desiredVoltage = 0;

  public Intake() {
    // Misc setup goes here
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(kIntakeInverted);
    config.smartCurrentLimit(kIntakeCurrentLimit);
    config.voltageCompensation(kNominalVoltage);
    config.idleMode(IdleMode.kCoast);
    // m_spark.setSmartCurrentLimit(kIntakeCurrentLimit);
    // m_spark.setInverted(kIntakeInverted);
    // m_spark.enableVoltageCompensation(kNominalVoltage);
    // m_spark.setIdleMode(IdleMode.kCoast);
    m_spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_subsystemNameCurrentState = IntakeStates.StateOff;
    m_subsystemNameRequestedState = IntakeStates.StateOff;
  }

  @Override
  public void periodic() {
    switch (m_subsystemNameRequestedState) {
      case StateOff:
        desiredVoltage = 0;
        break;
      case StateIntake:
        desiredVoltage = 5;
        break;
      case StateOuttake:
        desiredVoltage = -5;
        break;
    }
	
    runControlLoop();

    m_subsystemNameCurrentState = m_subsystemNameRequestedState;
  }

  public void runControlLoop() {
    m_spark.setVoltage(desiredVoltage);
  }
  
  // example of a "setter" method
  public void requestState(IntakeStates desiredState) {
    m_subsystemNameRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public IntakeStates getCurrentState() { 
    return m_subsystemNameCurrentState; 
  }

  // misc methods go here, getters and setters should follow above format
}
