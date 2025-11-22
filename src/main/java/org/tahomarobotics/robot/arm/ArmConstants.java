package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;


public class ArmConstants {

    // add real positions later. These are all in degrees
    public static final Angle LOW = Degrees.of(10);
    public static final Angle MID =  Degrees.of(10);;
    public static final Angle HIGH =  Degrees.of(10);;
    public static final Angle STOW =  Degrees.of(10);;

    // add real soft limits later. These are all in degrees
    public static final Angle MIN_POSITION =  Degrees.of(10);;
    public static final Angle MAX_POSITION =  Degrees.of(20);;


    // gear reduction
    public static final double GEAR_REDUCTION = 32;


    static final TalonFXConfiguration armMotorConfig() {

        return new TalonFXConfiguration()
        .withSlot0(
                new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        // add real PID constants later
                        .withKP(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0)
                        .withKG(0))

        .withMotorOutput(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(MIN_POSITION)

                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(MAX_POSITION)
                )

                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(GEAR_REDUCTION)

                );
        }



}
