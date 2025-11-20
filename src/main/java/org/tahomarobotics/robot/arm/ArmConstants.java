package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ArmConstants {


    public static final double LOW = 10;
    public static final double MID = 10;
    public static final double HIGH = 10;
    public static final double STOW = 10;

    public static final double MIN_POSITION = 10;
    public static final double MAX_POSITION = 20;


    // gear reduction
    public static final double GEAR_REDUCTION = 32;


    static final TalonFXConfiguration armMotorConfig() {

        return new TalonFXConfiguration()
        .withSlot0(
                new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        // add real PID constants later
                        .withKP(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0)
                        .withKG(0))

        .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(MIN_POSITION)

                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(MAX_POSITION)
                )

                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(GEAR_REDUCTION)

                );
        }



}
