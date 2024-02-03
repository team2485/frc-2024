package frc.robot.subsystems.NoteHandling;


import edu.wpi.first.math.controller.PIDController;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.*;


public class Shooter extends SubsystemBase {

    public enum ShooterStates{
        StateOff,
        StateMovingToRequestedState,
        StateDefault,
        StateSubwoofer,
        StatePodium,
        StateAmp,
    }

    public static ShooterStates m_shooterRequestedState;
    public static ShooterStates m_shooterCurrentState; 
    
    private final TalonFX m_talonRight = new TalonFX(kShooterRightPort);
    private final TalonFX m_talonLeft = new TalonFX(kShooterLeftPort);

    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

    private double desiredVelocity = kCoastVelocity;

    public Shooter (){

        var talonFXConfigs = new TalonFXConfiguration();


        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kSShooter;
        slot0Configs.kV = kVShooter;
        slot0Configs.kP = kPShooter;
        slot0Configs.kI = kIShooter;
        slot0Configs.kD = kDShooter; 


        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kCoastVelocity;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
    

    }


    @Override
    


    public void periodic() {
        switch (m_shooterRequestedState) {
          case StateOff:
            desiredVelocity = 0;
          case StateDefault:
            desiredVelocity = kCoastVelocity;
            break;
          case StateSubwoofer:
            desiredVelocity = 0;
            break;
          case StateAmp:
            desiredVelocity = 0;
            break;
          case StatePodium:
            desiredVelocity = 0;
            break;
        }
     
        runControlLoop();
    
        if (getError() < kShooterErrorTolerance)
          m_shooterCurrentState = m_shooterRequestedState;
        else
          m_shooterCurrentState = ShooterStates.StateMovingToRequestedState;  
        }
    
        public void runControlLoop() {
        // m_talon.setControl(request.withPosition(desiredPosition));
      }
    
      private double getVelocity() {
        return m_talonRight.getVelocity().getValue();
      }
    
      public double getError() {
        return Math.abs(getVelocity() - kCoastVelocity);
      }
     
      // example of a "setter" method
      public void requestState(ShooterStates requestedState) {
        m_shooterRequestedState = requestedState;
      }
     
      // example of a "getter" method
      public ShooterStates getCurrentState() {
        return m_shooterCurrentState;
      }
    
      // misc methods go here, getters and setters should follow above format
    }

    




    






  














