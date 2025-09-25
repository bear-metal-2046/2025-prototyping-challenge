package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Windmill extends SubsystemIF {
    //Arm elevator motors
    private static final TalonFX elevatorL = new TalonFX(RobotMap.WINDMILL_ELEVATOR_L);
    private static final TalonFX elevatorR = new TalonFX(RobotMap.WINDMILL_ELEVATOR_R);

    //Arm pivot motors
    private static final TalonFX topMotor = new TalonFX(RobotMap.WINDMILL_TOP_MOTOR);
    private static final TalonFX bottomMotor = new TalonFX(RobotMap.WINDMILL_BOTTOM_MOTOR);

    //Controllers
    private static final MotionMagicVelocityVoltage velocityController = new MotionMagicVelocityVoltage();

    private static Pose currentPose = Pose.LOW;

    Windmill() {
        RobustConfigurator.tryConfigureTalonFX("topMotor", topMotor, WindmillConstants.articulationMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("bottomMotor", bottomMotor, WindmillConstants.articulationMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("elevatorLMotor", elevatorL, WindmillConstants.elevatorMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("elevatorRMotor", elevatorR, WindmillConstants.elevatorMotorConfig);
        //set update frequency?
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
        //shoulder velocity = top velocity + bottom velocity
        //divide target velocity by two and rotate top and bottom motors by that value?
        topMotor.setControl(velocityController.withVelocity(targetVelocity / 2.0));
        bottomMotor.setControl(velocityController.withVelocity(targetVelocity / 2.0));
    }

    /**
     * Set the rotational velocity of the wrist joint
     * @param targetVelocity - target velocity, degrees per second
     */
    void setWristVelocity(double targetVelocity) {
        //wrist(target) velocity = top velocity - bottom velocity
        //bottom velocity = top velocity - target
        //top velocity is always max unless at rest, then?
        if (targetVelocity != 0) {
            topMotor.setControl(velocityController.withVelocity(WindmillConstants.ARTICULATION_MAX_VELOCITY));
            bottomMotor.setControl(velocityController.withVelocity(WindmillConstants.ARTICULATION_MAX_VELOCITY - targetVelocity));
        }

    }

    /**
     * Switch windmill between poses
     * @param newPose - target pose
     */
    void setPose(Pose newPose) {
        currentPose = newPose;
        setElevatorHeight(newPose.height);
        setShoulderMotorPosition(newPose.thetaShoulder);
        setWristMotorPosition(newPose.thetaWrist);
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
