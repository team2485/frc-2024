package frc.robot.subsystems.NoteHandling;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GeneralRollerConstants.*;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class GeneralRoller extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum GeneralRollerStates {
    StateOff,
    StateForward,
    StateReverse,
    StateForwardFast,
  }

  public GeneralRollerStates m_subsystemNameCurrentState;
  public GeneralRollerStates m_subsystemNameRequestedState;

  // You may need more than one motor
  private final CANSparkMax m_spark;
  // Units depend on the units of the setpoint() and calculate() methods. This example will use meters
  private double desiredVoltage = 0;

  public GeneralRoller(int port, boolean setInverted) {
    m_spark = new CANSparkMax(port, MotorType.kBrushless);

    m_spark.setSmartCurrentLimit(kGeneralRollerCurrentLimit);
    m_spark.setInverted(setInverted); 	
    m_spark.enableVoltageCompensation(kNominalVoltage);
    m_spark.setIdleMode(IdleMode.kBrake);

    m_subsystemNameCurrentState = GeneralRollerStates.StateOff;
    m_subsystemNameRequestedState = GeneralRollerStates.StateOff;
  }

  @Override
  public void periodic() {
    switch (m_subsystemNameRequestedState) {
      case StateOff:
        desiredVoltage = 0;
        break;
      case StateForward:
        desiredVoltage = 3;
        break;
      case StateReverse:
        desiredVoltage = -3;
        break;
      case StateForwardFast:
        desiredVoltage = 10;
        break;

    }
	
    runControlLoop();

    m_subsystemNameCurrentState = m_subsystemNameRequestedState;
  }

  public void runControlLoop() {
    m_spark.setVoltage(desiredVoltage);
  }
  
  // example of a "setter" method
  public void requestState(GeneralRollerStates desiredState) {
    m_subsystemNameRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public GeneralRollerStates getCurrentState() { 
    return m_subsystemNameCurrentState; 
  }

  // misc methods go here, getters and setters should follow above format
}
