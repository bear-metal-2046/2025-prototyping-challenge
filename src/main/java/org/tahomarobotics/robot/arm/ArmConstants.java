package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

public class ArmConstants {
    //Max speeds, radians/second
    static final double ELBOW_MAX_VELOCITY = 0.0;
    static final double ELBOW_MAX_ACCELERATION = 0.0;
    static final double ELBOW_MAX_JERK = 0.0;
    static final double WRIST_MAX_VELOCITY = 0.0;
    static final double WRIST_MAX_ACCELERATION = 0.0;
    static final double WRIST_MAX_JERK = 0.0;

    static final double TOP_MAX_VELOCITY = 0.0;
    static final double BOTTOM_MAX_VELOCITY = 0.0;
    static final double TOP_MAX_ACCELERATION = 0.0;
    static final double BOTTOM_MAX_ACCELERATION = 0.0;
    static final double TOP_MAX_JERK = 0.0;
    static final double BOTTOM_MAX_JERK = 0.0;

    static final TalonFXConfiguration topMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()
                            .withGravityType(GravityTypeValue.Arm_Cosine)
                            .withKP(0)
                            .withKD(0)
                            .withKS(0)
                            .withKV(0)
                            .withKA(0)
                            .withKG(0)
            ).withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.CounterClockwise_Positive)
            ).withMotionMagic(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(Units.radiansToRotations(TOP_MAX_VELOCITY))
                            .withMotionMagicAcceleration(Units.radiansToRotations(TOP_MAX_ACCELERATION))
                            .withMotionMagicJerk(Units.radiansToRotations(TOP_MAX_JERK))
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            //doesn't have free rotation as far as i know
                            .withContinuousWrap(false)
            ).withAudio(
                    new AudioConfigs()
                            .withBeepOnBoot(true)
                            .withBeepOnConfig(true)
            );

    static final TalonFXConfiguration bottomMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(
                    new Slot0Configs()
                            .withGravityType(GravityTypeValue.Arm_Cosine)
                            .withKP(0)
                            .withKD(0)
                            .withKS(0)
                            .withKV(0)
                            .withKA(0)
                            .withKG(0)
            ).withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.CounterClockwise_Positive)
            ).withMotionMagic(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(Units.radiansToRotations(BOTTOM_MAX_VELOCITY))
                            .withMotionMagicAcceleration(Units.radiansToRotations(BOTTOM_MAX_ACCELERATION))
                            .withMotionMagicJerk(Units.radiansToRotations(BOTTOM_MAX_JERK))
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            //doesn't have free rotation as far as i know
                            .withContinuousWrap(false)
            ).withAudio(
                    new AudioConfigs()
                            .withBeepOnBoot(true)
                            .withBeepOnConfig(true)
            );
}
