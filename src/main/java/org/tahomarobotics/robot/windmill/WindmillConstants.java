package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class WindmillConstants {
    //degrees per second
    static final double ARTICULATION_MAX_VELOCITY = 360;
    //degrees per second^2, will reach max velocity in two seconds
    static final double ARTICULATION_MAX_ACCELERATION = ARTICULATION_MAX_VELOCITY / 2;


    //todo: arbitrary, we don't know this yet
    static final double ARTICULATION_SENSOR_TO_MECHANISM_RATIO = 1;
    static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 1;


    //Pose constants (meters, degrees), also arbitrary
    static final double ELEVATOR_MIN_POSE = 0.1;
    static final double ELEVATOR_MID_POSE = 0.5;
    static final double ELEVATOR_MAX_POSE = 1;
    static final int SHOULDER_MIN_POSE = 45;
    static final int SHOULDER_MID_POSE = 45;
    static final int SHOULDER_MAX_POSE = 45;
    static final int WRIST_MIN_POSE = 45;
    static final int WRIST_MID_POSE = 45;
    static final int WRIST_MAX_POSE = 45;

    static final TalonFXConfiguration articulationMotorConfig = new TalonFXConfiguration()
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ARTICULATION_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ARTICULATION_MAX_ACCELERATION)
                    //keep acceleration constant
                    .withMotionMagicJerk(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    //todo: is this true? top motor and bottom motor are (my prediction) facing the same direction, so they should have the same inversion
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ARTICULATION_SENSOR_TO_MECHANISM_RATIO))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    //continuous wrapping: if you're at 0 degrees and you want to rotate in the positive direction to go to 270 degrees, the motor will go 90 degrees in the negative direction because that's shorter
                    //false because it'd mess with the differential i think
                    .withContinuousWrap(false));

    static final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration()
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ARTICULATION_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ARTICULATION_MAX_ACCELERATION)
                    //keep acceleration constant
                    .withMotionMagicJerk(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    //todo: is this true? top motor and bottom motor are (my prediction) facing the same direction, so they should have the same inversion
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ELEVATOR_SENSOR_TO_MECHANISM_RATIO))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    //continuous wrapping: if you're at 0 degrees and you want to rotate in the positive direction to go to 270 degrees, the motor will go 90 degrees in the negative direction because that's shorter
                    //false because it'd mess with the differential i think
                    .withContinuousWrap(false));
}
