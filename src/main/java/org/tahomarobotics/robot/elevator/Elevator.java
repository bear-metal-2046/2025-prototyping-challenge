package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;

public class Elevator {
    //Arm elevator motors
    private static final TalonFX elevatorL = new TalonFX(RobotMap.ELEVATOR_L);
    private static final TalonFX elevatorR = new TalonFX(RobotMap.ELEVATOR_R);

    //Arm pivot motors
    private static final TalonFX shoulderMotor = new TalonFX(RobotMap.ARM_SHOULDER_MOTOR);
    private static final TalonFX wristMotor = new TalonFX(RobotMap.ARM_WRIST_MOTOR);

    Elevator() {

    }

    //Pose enum
    private enum Pose {
        HIGH,
        MIDDLE,
        LOW
    }
}
