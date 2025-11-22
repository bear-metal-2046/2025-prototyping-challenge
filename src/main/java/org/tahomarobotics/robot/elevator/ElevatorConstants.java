package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.*;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.*;

//these constants are from robot 2025 and may need to be adjusted
public class ElevatorConstants {

    // Gear reduction
    public static final double ELEVATOR_GEAR_REDUCTION = 52d / 12d;

    //Pully radius and Circumference
    public static final Distance ELEVATOR_MAIN_PULLEY_RADIUS = Inch.of(1.1056);
    public static final Distance ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = ELEVATOR_MAIN_PULLEY_RADIUS.times(2 * Math.PI);

    //Pose locations
    public static final Distance ELEVATOR_MAX_POSE = Meters.of(1.035);
    public static final Distance ELEVATOR_TEST_POSE = Meters.of(1.0);


    //Elevator Max Motions
    public static final LinearVelocity ELEVATOR_MAX_VELOCITY = Meters.per(Second).of(2.0); //Meters/Sec
    public static final LinearAcceleration ELEVATOR_MAX_ACCELERATION = Meters.per(Second).per(Second).of(15.0); //Meters/Sec^2
    public static final Velocity<LinearAccelerationUnit> ELEVATOR_MAX_JERK = Meters.per(Second).per(Second).per(Second).of(200.0); // Meters/Sec^3

    //Elevator Voltages
    public static final Voltage ELEVATOR_ZERO_VOLTAGE = Volt.of(-1.0);
    public static final Distance ELEVATOR_MIN_POSE = Meters.of(0.01);

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
            ).withMotionMagic(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY.in(Meters.per(Second)))
                            .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION.in(Meters.per(Second).per(Second)))
                            .withMotionMagicJerk(ELEVATOR_MAX_JERK.in(Meters.per(Second).per(Second).per(Second)))
            ).withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitThreshold(ELEVATOR_MAX_POSE.in(Meters))
                            .withReverseSoftLimitThreshold(ELEVATOR_MIN_POSE.in(Meters))
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            .withContinuousWrap(false)
            ).withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Meters))
                    .withRotorToSensorRatio(1.0 / ELEVATOR_GEAR_REDUCTION)
                    .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            );
}