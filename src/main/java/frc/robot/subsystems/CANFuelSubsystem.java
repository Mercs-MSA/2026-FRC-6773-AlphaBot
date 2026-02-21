// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// SparkMax imports (commented out for now since we're using TalonFX instead)
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {

  //private final SparkMax LeftIntakeLauncher;
  //private final SparkMax RightIntakeLauncher;
  //private final SparkMax Indexer;

  private final TalonFX leftIntakeLauncher =
      new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID, "rio");

  private final TalonFX rightIntakeLauncher =
      new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, "rio");

  private final TalonFX indexer =
      new TalonFX(INDEXER_MOTOR_ID, "rio");

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0).withSlot(0);

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    /*
    LeftIntakeLauncher =
        new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher =
        new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    Indexer =
        new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);
    */

    TalonFXConfiguration launcherConfig =
        new TalonFXConfiguration();

    launcherConfig.CurrentLimits.SupplyCurrentLimit =
        LAUNCHER_MOTOR_CURRENT_LIMIT;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Coast;

    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftIntakeLauncher.getConfigurator().apply(launcherConfig);

    launcherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    rightIntakeLauncher.getConfigurator().apply(launcherConfig);

    TalonFXConfiguration indexerConfig =
        new TalonFXConfiguration();

    indexerConfig.CurrentLimits.SupplyCurrentLimit =
        INDEXER_MOTOR_CURRENT_LIMIT;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    indexerConfig.MotorOutput.NeutralMode =
        NeutralModeValue.Brake;

    // Basic velocity PID (tune later)
    indexerConfig.Slot0.kP = 0.1;
    indexerConfig.Slot0.kI = 0.0;
    indexerConfig.Slot0.kD = 0.0;
    indexerConfig.Slot0.kV = 0.12;

    indexer.getConfigurator().apply(indexerConfig);

    SmartDashboard.putNumber("Intaking feeder roller value",
        INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value",
        INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value",
        INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value",
        LAUNCHING_LAUNCHER_PERCENT);
  }

  /*
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(power);
  }
  */

  public void setIntakeLauncherRoller(double power) {
    leftIntakeLauncher.setControl(
        dutyCycleOut.withOutput(power));
    rightIntakeLauncher.setControl(
        dutyCycleOut.withOutput(power));
  }

  // Velocity is in Rotations Per Second (RPS)
  public void setFeederRoller(double velocityRPS) {

    // Old Spark version:
    //Indexer.set(power);

    indexer.setControl(
        velocityVoltage.withVelocity(velocityRPS));
  }

  public void stop() {

    leftIntakeLauncher.setControl(
        dutyCycleOut.withOutput(0));
    rightIntakeLauncher.setControl(
        dutyCycleOut.withOutput(0));
    indexer.setControl(
        dutyCycleOut.withOutput(0));
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(
        "Indexer Velocity (RPS)",
        indexer.getVelocity().getValueAsDouble());
  }
}
