package frc.robot.subsystems.NoteHandling;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;

import static frc.robot.Constants.GeneralRollerConstants.*;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class LeafBlower extends SubsystemBase {
    public enum LeafBlowerStates {
        StateOn,
        StateOff,
    }

    public static LeafBlowerStates m_subsystemNameCurrentState;
    public static LeafBlowerStates m_subsystemNameRequestedState;

    private final CANSparkMax m_spark;
    private double desiredVoltage = 0;

    public LeafBlower(int port, boolean setInverted) {
        m_spark = new CANSparkMax(port, MotorType.kBrushless);
        m_spark.setSmartCurrentLimit(kGeneralRollerCurrentLimit);
        m_spark.setInverted(setInverted);
        m_spark.enableVoltageCompensation(kNominalVoltage);
        m_spark.setIdleMode(IdleMode.kCoast);

        m_subsystemNameCurrentState = LeafBlowerStates.StateOff;
        m_subsystemNameRequestedState = LeafBlowerStates.StateOff;
    }

    @Override
    public void periodic(){
    switch (m_subsystemNameCurrentState){
        case StateOff:
            desiredVoltage = 0;
            break;
        case StateOn:
            desiredVoltage = 12;
            break;
        }
        runControlLoop();

        m_subsystemNameCurrentState = m_subsystemNameRequestedState;
    }
    


    public void runControlLoop(){
        m_spark.setVoltage(desiredVoltage);
    }

     // example of a "setter" method
    public void requestState(LeafBlowerStates desiredState) {
        m_subsystemNameRequestedState = desiredState;
    }

    public LeafBlowerStates getCurrentState() { 
        return m_subsystemNameCurrentState; 
      }
 

}
