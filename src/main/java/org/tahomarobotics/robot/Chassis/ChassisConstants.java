package org.tahomarobotics.robot.Chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class ChassisConstants {
    public static final double wheelbase = 20.75 /* Need to get the real number*/;
    public static final double trackwidth = 20.75/* Need to get the real number*/;

    public static final double steerRatio = (1 /(150.0 /7)); /*Gotten from the Swerve Drive Specialists MK4i swerve module */


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
}




