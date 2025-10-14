package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

//these constants are from robot 2025 and may need to be adjusted
public class ElevatorConstants {

    // Gearing
    public static final double ELEVATOR_GEAR_REDUCTION = 52d / 12d;

    // Pulley
    public static final double ELEVATOR_MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * ELEVATOR_MAIN_PULLEY_RADIUS;

    // TalonFX configuration for elevator
    static final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()
                            .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.CounterClockwise_Positive)
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            .withContinuousWrap(false)
            ).withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE)
                    .withRotorToSensorRatio(1.0 / ELEVATOR_GEAR_REDUCTION)
                    .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            );
}