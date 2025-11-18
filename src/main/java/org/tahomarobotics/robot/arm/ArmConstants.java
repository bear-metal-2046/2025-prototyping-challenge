package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;



public class ArmConstants {



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
                        .withInverted(InvertedValue.CounterClockwise_Positive));



        }



}
