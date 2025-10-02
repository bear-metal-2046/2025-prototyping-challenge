package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class ChassisConstants {
    // Configuration constants
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final boolean DRIVE_MOTOR_INVERTED = false;
    public static final boolean STEER_MOTOR_INVERTED = true;
    public static final double DRIVE_K_P = 0.1;
    public static final double DRIVE_K_D = 0.0;
    public static final double STEER_K_P = 1.0;
    public static final double STEER_K_D = 0.0;

    // Returns TalonFXConfiguration for drive motor
    public static TalonFXConfiguration createDriveMotorConfiguration(int encoderId) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Slot0.kP = DRIVE_K_P;
        config.Slot0.kD = DRIVE_K_D;
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
        // Additional drive config as needed
        return config;
    }

    // Returns TalonFXConfiguration for steer motor, using encoder Id for feedback
    public static TalonFXConfiguration createSteerMotorConfiguration(int encoderId) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Slot0.kP = STEER_K_P;
        config.Slot0.kD = STEER_K_D;
        config.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = encoderId;

        // Additional steer config as needed
        return config;
    }
}
