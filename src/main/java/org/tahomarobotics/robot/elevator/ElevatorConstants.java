package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

//these constants are from robot 2025 and may need to be adjusted
public class ElevatorConstants {

    // Gearing
    //todo: placeholder value, replace with real one when available
    public static final double ELEVATOR_GEAR_REDUCTION = 12d / 52d;

    // Pulley
    public static final double ELEVATOR_MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * ELEVATOR_MAIN_PULLEY_RADIUS;

    // Poses
    public static final double ELEVATOR_MAX_POSE = 1.035 + Units.inchesToMeters(6); // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters
    public static final double ELEVATOR_MID_POSE = 0.45; // Meters

    // Tolerances
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.005; // Meters
    public static final double ELEVATOR_VELOCITY_TOLERANCE = 0.01; // Meters / sec

    // Motion
    public static final double ELEVATOR_MAX_VELOCITY = 2.0; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = 15.0; // Meters / sec^2
    public static final double ELEVATOR_MAX_JERK = 200; // Meters / sec^3

    // TalonFX PID slots
    static final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()
                            .withGravityType(GravityTypeValue.Elevator_Static)
                            // SysId'd 1/17/2025
                            .withKP(50.438)
                            .withKI(100)
                            .withKS(0.097499)
                            .withKV(3.3441)
                            .withKA(0.16116)
                            .withKG(0.084958)
            ).withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.CounterClockwise_Positive)
            ).withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(false)
                            .withSupplyCurrentLimitEnable(false)
            ).withMotionMagic(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                            .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                            .withMotionMagicJerk(ELEVATOR_MAX_JERK)
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            .withContinuousWrap(false)
            ).withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE)
                    .withRotorToSensorRatio(1.0 / ELEVATOR_GEAR_REDUCTION)
            ).withAudio(
                    new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
            );
}