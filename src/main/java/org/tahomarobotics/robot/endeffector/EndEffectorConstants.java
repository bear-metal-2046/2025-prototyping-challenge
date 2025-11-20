package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.Volts;

public class EndEffectorConstants {
    public static final Voltage MAXIMUM_VOLTAGE = Volts.of(6);





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
