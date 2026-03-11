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

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {

    // private final SparkMax LeftIntakeLauncher;
    // private final SparkMax RightIntakeLauncher;
    // private final SparkMax Indexer;

    public enum fuelSubsystemState {
        IDLE,
        WARMING,
        INTAKING,
        EJECTING,
        SHOOTING,
    }

    private fuelSubsystemState currentState = fuelSubsystemState.IDLE;

    private final CANBus canBus = new CANBus("rio");

    private final TalonFX leftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID, canBus);

    private final TalonFX rightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, canBus);

    private final TalonFX indexer = new TalonFX(INDEXER_MOTOR_ID, canBus);

    private final VelocityVoltage velocityVoltageLeft = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltageRight = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltageIndex = new VelocityVoltage(0).withSlot(0);

    private final TalonFXSimState motorSimLeft = leftIntakeLauncher.getSimState();
    private final TalonFXSimState motorSimRight = rightIntakeLauncher.getSimState();
    private final TalonFXSimState motorSimIndex = indexer.getSimState();

    /* Creates a new CANFuelSubsystem. */
    public CANFuelSubsystem() {
        /*
         * LeftIntakeLauncher =
         * new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
         * RightIntakeLauncher =
         * new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
         * Indexer =
         * new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);
         */

        TalonFXConfiguration leftlauncherConfig = new TalonFXConfiguration();

        leftlauncherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
        leftlauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftlauncherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftlauncherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftlauncherConfig.Slot0.kP = 0;
        leftlauncherConfig.Slot0.kI = 0.0;
        leftlauncherConfig.Slot0.kD = 0.0;
        leftlauncherConfig.Slot0.kV = 0.12;

        leftIntakeLauncher.getConfigurator().apply(leftlauncherConfig);

        TalonFXConfiguration rightLauncherConfig = new TalonFXConfiguration();

        rightLauncherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
        rightLauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightLauncherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightLauncherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightLauncherConfig.Slot0.kP = 0;
        rightLauncherConfig.Slot0.kI = 0;
        rightLauncherConfig.Slot0.kD = 0;
        rightLauncherConfig.Slot0.kV = 0.12;

        rightIntakeLauncher.getConfigurator().apply(rightLauncherConfig);

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

        indexerConfig.CurrentLimits.SupplyCurrentLimit = INDEXER_MOTOR_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Basic velocity PID (tune later)
        indexerConfig.Slot0.kP = 0;
        indexerConfig.Slot0.kI = 0;
        indexerConfig.Slot0.kD = 0;
        indexerConfig.Slot0.kV = 0.12;

        indexer.getConfigurator().apply(indexerConfig);
    }

    /*
     * public void setIntakeLauncherRoller(double power) {
     * LeftIntakeLauncher.set(power);
     * RightIntakeLauncher.set(power);
     * }
     */

    private void setIntakeLauncherRoller(double speed) {
        leftIntakeLauncher.setControl(
                velocityVoltageLeft.withVelocity(speed));
        rightIntakeLauncher.setControl(
                velocityVoltageRight.withVelocity(speed));
    }

    private void setIndexer(double speed) {
        indexer.setControl(
            velocityVoltageIndex.withVelocity(speed)
        );
    }

    public void stateControl(fuelSubsystemState state) {
        if (state == fuelSubsystemState.SHOOTING) {
            System.out.println("No ow ow it hurts no I can't change to that state here no ow");
        } else if (currentState == fuelSubsystemState.SHOOTING && state == fuelSubsystemState.WARMING)
        {
            System.out.println("No I'm already doing that");
        }
        {
            currentState = state;
        }
    }

    public void cancelShooting() {
        if (currentState == fuelSubsystemState.SHOOTING || currentState == fuelSubsystemState.WARMING) {
            currentState = fuelSubsystemState.IDLE;
        }
    }

    private void stop() {
        leftIntakeLauncher.setControl(
                velocityVoltageLeft.withVelocity(0));
        rightIntakeLauncher.setControl(
                velocityVoltageRight.withVelocity(0));
        indexer.setControl(
                velocityVoltageIndex.withVelocity(0));
    }

    private double GetShooterVelocity() {
        return (rightIntakeLauncher.getVelocity().getValueAsDouble()
                + leftIntakeLauncher.getVelocity().getValueAsDouble()) / 2;
    }

    @Override
    public void periodic() {
        // Our state machine, my pride and joy, beautiful summer child
        switch (currentState) {
            case IDLE:
                // Arrest all motors while IDLE
                stop();
                break;
            case WARMING:
                // WARMING up motors (spin up flywheel, wait for it, and switch states)
                setIntakeLauncherRoller(LAUNCHING_LAUNCHER_SPEED);

                // If speed is right, break out into SHOOTING mode
                // SHOOTING mode normally can't be accessed except by first warming up
                if (GetShooterVelocity() >= 20) {
                    currentState = fuelSubsystemState.SHOOTING;
                }

                // Arrest indexer (just in case)
                setIndexer(0);
                break;
            case SHOOTING:
                // Activate indexer to funnel balls into the flywheel
                setIndexer(INDEXER_TRANSFER_SPEED);

                // Keep the shooter motor active (just in case)
                setIntakeLauncherRoller(LAUNCHING_LAUNCHER_SPEED);
                break;
            case INTAKING:
                // Arrest indexer (just in case)
                setIndexer(0);

                // Use slower intaking speed
                setIntakeLauncherRoller(INTAKE_INTAKING_SPEED);
                break;
            case EJECTING:
                // Intake but backwards
                // Arrest indexer (just in case)
                setIndexer(0);

                // Use eject speed
                setIntakeLauncherRoller(INTAKE_EJECT_SPEED);
                break;
            default:
                stop();
        }

        SmartDashboard.putNumber(
                "Indexer Velocity (RPS)",
                indexer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake velocity (left)",
                leftIntakeLauncher.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake velocity (right)",
                rightIntakeLauncher.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // Set the bus voltage so the motor has power in the sim
        motorSimLeft.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorSimRight.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorSimIndex.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Get the current position from the motor's encoder (StatusSignal)
        double currentVelLeft = leftIntakeLauncher.getVelocity().getValueAsDouble();
        double currentVelRight = rightIntakeLauncher.getVelocity().getValueAsDouble();
        double currentVelIndex = indexer.getVelocity().getValueAsDouble();

        // Get the voltage being sent to the motor (Primitive double)
        double appliedVoltageLeft = motorSimLeft.getMotorVoltage();
        double appliedVoltageRight = motorSimRight.getMotorVoltage();
        double appliedVoltageIndex = motorSimIndex.getMotorVoltage();

        // Update the simulated position
        // Add a small amount based on voltage to "fake" movement
        motorSimLeft.setRotorVelocity(currentVelLeft + (appliedVoltageLeft * 0.05));
        motorSimRight.setRotorVelocity(currentVelRight + (appliedVoltageRight * 0.05));
        motorSimIndex.setRotorVelocity(currentVelIndex + (appliedVoltageIndex * 0.05));
    }
}