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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private final SparkMax climberMotor;

  private final TalonFX climberMotor =
      new TalonFX(CLIMBER_MOTOR_ID, "rio");


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

    climberMotor.getConfigurator().apply(climbConfig);
  }

  public void setClimber(double power) {
    climberMotor.setControl(
        new DutyCycleOut(power));  
  }

 
  public void stop() {
    climberMotor.setControl(
        new DutyCycleOut(0));  
  }

  @Override
  public void periodic() {
   
  }
}