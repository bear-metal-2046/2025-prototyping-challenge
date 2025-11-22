package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


public class ChassisConstants {


    public static final double STEER_RATIO = 50d / 14d * 60d / 10d;
    public static final double DRIVE_RATIO = (16.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);


    public static TalonFXConfiguration createDriveMotorConfig() {

        return new TalonFXConfiguration()

                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(InvertedValue.Clockwise_Positive)
                ).withFeedback(
                        new FeedbackConfigs()
                                .withRotorToSensorRatio(DRIVE_RATIO)
                );
    }


    public static TalonFXConfiguration createSteerMotorConfig(int encoderId, double steerRatio) {
        return new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(InvertedValue.Clockwise_Positive)
                ).withFeedback(new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(encoderId)
                        .withRotorToSensorRatio(steerRatio)
                );
    }


    public static CANcoderConfiguration createEncoderConfig() {
        return new CANcoderConfiguration()
                .withMagnetSensor(
                        new MagnetSensorConfigs()
                                .withAbsoluteSensorDiscontinuityPoint(1)
                                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                );
    }
}


