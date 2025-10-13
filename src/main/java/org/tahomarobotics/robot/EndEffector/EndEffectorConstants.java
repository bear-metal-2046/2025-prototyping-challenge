package org.tahomarobotics.robot.EndEffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorConstants {




    public static final double GEAR_RATIO = 10.0; // this is a place holder not the acutual gear ratio
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            // SysId'd 02/11
            .withSlot0(new Slot0Configs()
                    .withKP(1.048663)
                    .withKS(0.05988)
                    .withKV(0.37575)
                    .withKA(0.0053009))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(40)
                    .withMotionMagicAcceleration(100)
                    .withMotionMagicJerk(5000))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / GEAR_RATIO))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));


}
