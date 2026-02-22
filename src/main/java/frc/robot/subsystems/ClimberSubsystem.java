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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//Since we are not using duty cycle output, we don't need to import it
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.CANBus;

public class ClimberSubsystem extends SubsystemBase {

  // private final SparkMax climberMotor;

  private CANBus rio = new CANBus("rio");
  private final TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID, rio);
  private final PositionVoltage climberPosition = new PositionVoltage(0.0).withSlot(0);
  PositionVoltage currentPosition = new PositionVoltage(0.0).withSlot(0); // this is not necessary; the current position can be tracked with a double, if at all

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

    climbConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;

    climberMotor.getConfigurator().apply(climbConfig);
  }

  public void setClimber(double power) { // rename this input parameter to "position"
    currentPosition = currentPosition.withPosition(power);
    climberMotor.setControl(currentPosition);
  } 

  // add a method to return the position (as a double) of the climber 
 
  public void stop() {
    currentPosition = currentPosition.withPosition(0.0); // this is very dangerous. It doesn't stop the motor, it would immediately send it back to the start. We don't need a stop method
    climberMotor.setControl(currentPosition);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("climber position", climberMotor.getMotorVoltage().getValueAsDouble());
   
  }
}