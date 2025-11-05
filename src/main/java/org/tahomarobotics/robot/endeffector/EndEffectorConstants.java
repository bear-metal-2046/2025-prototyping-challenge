package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.RobotMap;

public class EndEffectorConstants {




    public static final double GEAR_RATIO = 10.0; // this is a placeholder not the acutual gear ratio
    static final TalonFXConfiguration endEffectorMotorConfig = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()
                            .withGravityType(GravityTypeValue.Arm_Cosine)
            )
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
