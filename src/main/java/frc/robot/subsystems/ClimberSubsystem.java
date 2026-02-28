package frc.robot.subsystems;

// SparkMax imports (commented out for now since we're using TalonFX instead)

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkMax;

import static frc.robot.Constants.ClimbConstatns.CLIMBER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstatns.CLIMBER_MOTOR_ID;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
//Since we are not using duty cycle output, we don't need to import it
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private final SparkMax climberMotor;

  private CANBus rio = new CANBus("rio");
  private final TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID, rio);
  private final PositionVoltage climberPosition = new PositionVoltage(0.0).withSlot(0);
  private PositionVoltage currentPosition = climberPosition;
  public ClimberSubsystem() {
    /*
    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    */

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    
    climbConfig.CurrentLimits.SupplyCurrentLimit =
        CLIMBER_MOTOR_CURRENT_LIMIT;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // you must add more settings here for control loops. This includes at least proportional (P) gain. Look at the example CTRE Position control Java code to see what I mean

    var slot0Configs = new Slot0Configs();
  slot0Configs.kP = 24; // An error of 0.5 rotations results in 12 V output
  slot0Configs.kI = 0; // no output for integrated error
  slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

climberMotor.getConfigurator().apply(slot0Configs);
    climbConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;

    climberMotor.getConfigurator().apply(climbConfig);
  }

  public void setClimber(double position) { 
    currentPosition = currentPosition.withPosition(position);
    climberMotor.setControl(currentPosition);
  } 
 
  public double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  /* public void stop() {
    currentPosition = currentPosition.withPosition(0.0); // this is very dangerous. It doesn't stop the motor, it would immediately send it back to the start. We don't need a stop method
    climberMotor.setControl(currentPosition);
  } */ 
 //We don't need a stop method since we are using position control. If we wanted to stop the motor, we would just set the position to the current position, which would cause the motor to hold its position right where it is currently (the desired climber height).

  @Override
  public void periodic() {

    SmartDashboard.putNumber("climber position", climberMotor.getMotorVoltage().getValueAsDouble());
   
  }
}