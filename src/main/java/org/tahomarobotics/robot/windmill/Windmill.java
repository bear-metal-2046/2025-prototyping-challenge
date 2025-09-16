package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Windmill extends SubsystemIF {
    //Arm elevator motors
    private static final TalonFX elevatorL = new TalonFX(RobotMap.WINDMILL_ELEVATOR_L);
    private static final TalonFX elevatorR = new TalonFX(RobotMap.WINDMILL_ELEVATOR_R);

    //Arm pivot motors
    private static final TalonFX shoulderMotor = new TalonFX(RobotMap.WINDMILL_SHOULDER_MOTOR);
    private static final TalonFX wristMotor = new TalonFX(RobotMap.WINDMILL_WRIST_MOTOR);

    private static Pose currentPose = Pose.LOW;

    Windmill() {

    }

    @Override
    public SubsystemIF initialize() {
        setPose(currentPose);
        return this;
    }

    /**
     * Move elevator to a desired height
     * @param target - target elevator height in meters
     */
    //todo write this
    void setElevatorHeight(double target) {

    }

    /**
     * Rotate shoulder motor
     * @param targetTheta - desired rotational position of shoulder motor
     */
    void setShoulderMotorPosition(double targetTheta) {

    }

    /**
     * Rotate wrist motor
     * @param targetTheta - desired rotational position of wrist motor
     */
    void setWristMotorPosition(double targetTheta) {

    }

    /**
     * Switch windmill between poses
     * @param newPose - target pose
     */
    void setPose(Pose newPose) {

    }

    //Pose enum (arbitrary poses for now)
    private enum Pose {
        HIGH(1, 45, 45),
        MIDDLE(0.5, 45, 45),
        LOW(0.1, 45, 45);

        double height;
        int thetaShoulder;
        int thetaWrist;
        Pose(double height, int thetaShoulder, int thetaWrist) {
            this.height = height;
            this.thetaShoulder = thetaShoulder;
            this.thetaWrist = thetaWrist;
        }
    }
}
