package frc.robot.subsystems.Climb;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

import frc.robot.commands.Interpolation.InterpolatingTable;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum ClimberStates {
    StateNotPrimed,
    StatePrimed,
    StateUp,
    StateDownVoltage,
    StateDownPosition,
    StateMovingToRequestedState
  }

  public static ClimberStates m_ClimberCurrentState;
  public static ClimberStates m_ClimberRequestedState;

  GenericEntry climberPosition;

  // You may need more than one motor
  private final TalonFX m_talonLeft = new TalonFX(kClimberLeftPort, "Mast");
  private final TalonFX m_talonRight = new TalonFX(kClimberRightPort, "Mast");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;
  private double desiredVoltage = 0;

  private boolean leftSet = false;
  private boolean rightSet = false;

  Debouncer m_climberLeftDebouncer;
  Debouncer m_climberRightDebouncer;

  public Climber() {
    climberPosition = Shuffleboard.getTab("Swerve").add("Current", 0).getEntry();

    m_climberLeftDebouncer = new Debouncer(1, DebounceType.kBoth);
    m_climberRightDebouncer = new Debouncer(1, DebounceType.kBoth);

    // Misc setup goes here
    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSClimberLeft;
    slot0Configs.kV = kVClimber;
    slot0Configs.kP = kPClimberLeft;
    slot0Configs.kI = kIClimber;
    slot0Configs.kD = kDClimber;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kClimberCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kClimberAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = kClimberJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kClimberClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;

    var currentConfigs = talonFXConfigs.CurrentLimits;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = kCurrentLimit;

    m_talonLeft.getConfigurator().apply(talonFXConfigs);

    slot0Configs.kS = kSClimberRight;
    slot0Configs.kP = kPClimberRight;

    if (kClimberClockwisePositive) {
      motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    else motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    m_talonRight.getConfigurator().apply(talonFXConfigs);

    m_ClimberCurrentState = ClimberStates.StateNotPrimed;
    m_ClimberRequestedState = ClimberStates.StateNotPrimed;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_talonLeft.setPosition(0);
    m_talonRight.setPosition(0);
  }

  @Override
  public void periodic() {
    climberPosition.setDouble(m_talonLeft.getTorqueCurrent().getValueAsDouble());

    switch (m_ClimberRequestedState) {
      case StateUp:
        desiredPosition = 2;
        desiredVoltage = 0;
        break;
      case StateDownVoltage:
        desiredPosition = 0;
        desiredVoltage = -5;
        break;
      case StateDownPosition:
        desiredPosition = -0.25;
        desiredVoltage = 0;
        break;
    }
 
    runControlLoop();

    if (getErrorLeft() < kClimberErrorTolerance && getErrorRight() < kClimberErrorTolerance)
      m_ClimberCurrentState = m_ClimberRequestedState;
    else
      m_ClimberCurrentState = ClimberStates.StateMovingToRequestedState;  
  }

  public void runControlLoop() {    
    if (desiredVoltage != 0) {
        m_talonLeft.setVoltage(desiredVoltage);
        if (leftSet) m_talonLeft.setVoltage(0);
        m_talonRight.setVoltage(desiredVoltage);
        if (rightSet) m_talonRight.setVoltage(0);
    }
    else {
      m_talonLeft.setControl(request.withPosition(desiredPosition));
      m_talonRight.setControl(request.withPosition(desiredPosition));
    }
  }

  private double getPositionLeft() {
    return m_talonLeft.getPosition().getValue();
  }


  public double getErrorLeft() {
    if (getRequestedState() != ClimberStates.StateDownVoltage)
      return Math.abs(getPositionLeft() - desiredPosition);
    else {
      if (m_climberLeftDebouncer.calculate(Math.abs(m_talonLeft.getTorqueCurrent().getValueAsDouble()) < kCurrentLimitThreshold) && !leftSet || getPositionLeft() > 1.9) 
        return kClimberErrorTolerance + 1;
      else {
        leftSet = true;
        return 0;
      }
    }
  }

  private double getPositionRight() {
    return m_talonRight.getPosition().getValue();
  }

  public double getErrorRight() {
    if (getRequestedState() != ClimberStates.StateDownVoltage)
      return Math.abs(getPositionRight() - desiredPosition);
    else {
      if (m_climberRightDebouncer.calculate(Math.abs(m_talonRight.getTorqueCurrent().getValueAsDouble()) < kCurrentLimitThreshold) && !rightSet || getPositionRight() > 1.9) 
        return kClimberErrorTolerance + 1;
      else {
        rightSet = true;
        return 0;
      }
    }
  }

  public void zeroClimber() {
    m_talonLeft.setPosition(0);
    m_talonRight.setPosition(0);
  }
 
  // example of a "setter" method
  public void requestState(ClimberStates requestedState) {
    m_ClimberRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public ClimberStates getCurrentState() {
    return m_ClimberCurrentState;
  }

  public ClimberStates getRequestedState() {
    return m_ClimberRequestedState;
  }

  // misc methods go here, getters and setters should follow above format
}
