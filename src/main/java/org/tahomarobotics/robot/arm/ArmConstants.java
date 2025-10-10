package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

public class ArmConstants {
    //output rotations/input rotations
    static final double TOP_MOTOR_GEAR_RATIO = 24.0 / 1.0;
    static final double BOTTOM_MOTOR_GEAR_RATIO = 24.0 / 1.0;

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
                            .withInverted(InvertedValue.Clockwise_Positive)
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
                            .withInverted(InvertedValue.Clockwise_Positive)
            ).withClosedLoopGeneral(
                    new ClosedLoopGeneralConfigs()
                            //also doesn't have free rotation afaik
                            .withContinuousWrap(false)
            ).withAudio(
                    new AudioConfigs()
                            .withBeepOnBoot(true)
                            .withBeepOnConfig(true)
            );
}
