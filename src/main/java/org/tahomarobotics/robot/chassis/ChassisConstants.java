package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;



public class ChassisConstants {


    // Current Limits

    private static final double DRIVE_SUPPLY_CURRENT_LIMIT = 80;
    private static final double DRIVE_STATOR_CURRENT_LIMIT = 160;



    public static final double steerRatio = (1 /(150.0 /7)); /*Gotten from the Swerve Drive Specialists MK4i swerve module */



    public static final CurrentLimitsConfigs teleopConfiguration =

            new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT);


    public static TalonFXConfiguration createDriveMotorConfig() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        return new TalonFXConfiguration()

                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(InvertedValue.Clockwise_Positive)
                ).withCurrentLimits(
                        teleopConfiguration
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