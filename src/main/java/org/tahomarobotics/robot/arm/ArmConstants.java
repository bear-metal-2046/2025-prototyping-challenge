package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

public class ArmConstants {
    //Ratios
    static final double SENSOR_TO_MECHANISM_RATIO = 0;

    //output rotations/input rotations
    static final double MOTOR_GEAR_RATIO = 60d / 10d * 48d / 12d;

    static final TalonFXConfiguration topMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.Clockwise_Positive)
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            //ensures that arm moves within its limits rather than attempting to rotate into the robot
                            .withContinuousWrap(false)
            ).withAudio(
                    new AudioConfigs()
                            .withBeepOnBoot(true)
                            .withBeepOnConfig(true)
            ).withFeedback(
                    new FeedbackConfigs()
                            .withFeedbackRemoteSensorID(RobotMap.ARM_TOP_ENCODER)
                            //post-STMR unit: output rotations
                            .withSensorToMechanismRatio(MOTOR_GEAR_RATIO)
            );

    static final TalonFXConfiguration bottomMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withInverted(InvertedValue.CounterClockwise_Positive)
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            //also doesn't have free rotation afaik
                            .withContinuousWrap(false)
            ).withAudio(
                    new AudioConfigs()
                            .withBeepOnBoot(true)
                            .withBeepOnConfig(true)
            ).withFeedback(
                    new FeedbackConfigs()
                            .withFeedbackRemoteSensorID(RobotMap.ARM_BOTTOM_ENCODER)
                            .withSensorToMechanismRatio(MOTOR_GEAR_RATIO)
            );
}
