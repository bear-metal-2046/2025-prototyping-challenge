package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

//these constants are from robot 2025 and may need to be adjusted
public class ElevatorConstants {

    // Gear reduction
    public static final double ELEVATOR_GEAR_REDUCTION = 52d / 12d;

    //Pully radius and Circumference
    public static final Distance ELEVATOR_MAIN_PULLEY_RADIUS = Inch.of(1.1056);
    public static final Distance ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = ELEVATOR_MAIN_PULLEY_RADIUS.times(2*Math.PI);

    //Pose locations
    public static final Distance ELEVATOR_MIN_POSE = Meters.of (0.01);

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
                    .withSensorToMechanismRatio(1.0 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Inch))
                    .withRotorToSensorRatio(1.0 / ELEVATOR_GEAR_REDUCTION)
                    .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            );
}