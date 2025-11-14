package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.RobotMap;

public class EndEffectorConstants {





    static final TalonFXConfiguration endEffectorMotorConfig = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()

            )
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
)

            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));



}
