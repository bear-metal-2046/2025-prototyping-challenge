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
    private static final TalonFX topMotor = new TalonFX(RobotMap.WINDMILL_TOP_MOTOR);
    private static final TalonFX bottomMotor = new TalonFX(RobotMap.WINDMILL_BOTTOM_MOTOR);

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
     * Set the rotational velocity of the shoulder joint
     * @param targetVelocity - target velocity, degrees per second
     */
    void setShoulderVelocity(double targetVelocity) {
        //shoulder velocity = top velocity - bottom velocity
    }

    /**
     * Set the rotational velocity of the wrist joint
     * @param targetVelocity - target velocity, degrees per second
     */
    void setWristVelocity(double targetVelocity) {
        //wrist velocity = bottom velocity - top velocity
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
