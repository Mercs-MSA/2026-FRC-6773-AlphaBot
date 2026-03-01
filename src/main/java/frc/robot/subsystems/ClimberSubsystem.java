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
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private final SparkMax climberMotor;

  private CANBus rio = new CANBus("rio");
  private final TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID, rio);
  private final PositionVoltage climberPosition = new PositionVoltage(0.0).withSlot(0);
  private PositionVoltage currentPosition = climberPosition;

  //Simulation method
  private final TalonFXSimState motorSim = climberMotor.getSimState();

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
    climbConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;

    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 24; // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    climberMotor.getConfigurator().apply(climbConfig);
    climberMotor.getConfigurator().apply(slot0Configs);
  }

  public void setClimber(double position) { 
    currentPosition = climberPosition.withPosition(position);
    climberMotor.setControl(currentPosition);
  } 
  
  public void goHome() {
    climberMotor.setControl(climberPosition.withPosition(0));
  }

  public void goLevelOne() {
    climberMotor.setControl(climberPosition.withPosition(15));
  }
  public double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Climber Actual Position", climberMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Target Position", climberPosition.Position);
    SmartDashboard.putNumber("Climber Voltage", climberMotor.getMotorVoltage().getValueAsDouble());
  }

@Override
public void simulationPeriodic() {
  //Set the bus voltage so the motor has power in the sim
  motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

  //Get the current position from the motor's encoder (StatusSignal)
  double currentPos = climberMotor.getPosition().getValueAsDouble();

  //Get the voltage being sent to the motor (Primitive double)
  double appliedVoltage = motorSim.getMotorVoltage();

  //Update the simulated position
  //Add a small amount based on voltage to "fake" movement
  motorSim.setRawRotorPosition(currentPos + (appliedVoltage * 0.01));
}
}