package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;

public class Windmill {
    //Arm elevator motors
    private static final TalonFX elevatorL = new TalonFX(RobotMap.WINDMILL_ELEVATOR_L);
    private static final TalonFX elevatorR = new TalonFX(RobotMap.WINDMILL_ELEVATOR_R);

    //Arm pivot motors
    private static final TalonFX shoulderMotor = new TalonFX(RobotMap.WINDMILL_SHOULDER_MOTOR);
    private static final TalonFX wristMotor = new TalonFX(RobotMap.WINDMILL_WRIST_MOTOR);

    Windmill() {

    }

    //Pose enum
    private enum Pose {
        HIGH,
        MIDDLE,
        LOW
    }
}
