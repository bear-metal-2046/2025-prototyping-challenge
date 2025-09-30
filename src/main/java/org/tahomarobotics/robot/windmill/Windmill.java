package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

//TODO: how to register subsystem if it has to be package-private?
public class Windmill extends SubsystemIF {
    //Arm elevator motors
    private static final TalonFX elevatorL = new TalonFX(RobotMap.WINDMILL_ELEVATOR_L);
    private static final TalonFX elevatorR = new TalonFX(RobotMap.WINDMILL_ELEVATOR_R);

    //Arm pivot motors
    private static final TalonFX topMotor = new TalonFX(RobotMap.WINDMILL_TOP_MOTOR);
    private static final TalonFX bottomMotor = new TalonFX(RobotMap.WINDMILL_BOTTOM_MOTOR);

    //Controllers
    private static final MotionMagicVelocityVoltage velocityController = new MotionMagicVelocityVoltage(0);
    private static final MotionMagicVoltage positionController = new MotionMagicVoltage(0);

    //Tracker variables
    private static Pose currentPose = Pose.LOW;
    private static double height = Pose.LOW.height;

    Windmill() {
        RobustConfigurator.tryConfigureTalonFX("topMotor", topMotor, WindmillConstants.articulationMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("bottomMotor", bottomMotor, WindmillConstants.articulationMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("elevatorL", "elevatorR", elevatorL, elevatorR, WindmillConstants.elevatorMotorConfig, true);
        //set update frequency?
    }

    @Override
    public SubsystemIF initialize() {
        setPose(currentPose);
        return this;
    }

    /**
     * Move elevator to a desired height
     * @param targetHeight - target elevator height in meters
     */
    //todo write this
    void setElevatorHeight(double targetHeight) {
        //clamp target height just in case
        targetHeight = MathUtil.clamp(targetHeight, WindmillConstants.ELEVATOR_MIN_POSE, WindmillConstants.ELEVATOR_MAX_POSE);
        //r motor runs in reverse tandem with l motor
        elevatorL.setControl(positionController.withPosition(targetHeight));
        //what if the elevator fails to reach its target height?
        height = targetHeight;
    }

    //todo: should probably clamp position like with elevator?
    /**
     * Rotate shoulder motor
     * @param targetTheta - desired rotational position of shoulder motor
     */
    void setShoulderPosition(double targetTheta) {

    }

    /**
     * Rotate wrist motor
     * @param targetTheta - desired rotational position of wrist motor
     */
    void setWristPosition(double targetTheta) {

    }

    /**
     * Set the rotational velocity of the shoulder joint
     * @param targetVelocity - target velocity, degrees per second
     */
    void setShoulderVelocity(double targetVelocity) {
        //shoulder velocity = top velocity + bottom velocity
        //divide target velocity by two and rotate top and bottom motors by that value?
        if (targetVelocity != 0) {
            topMotor.setControl(velocityController.withVelocity(targetVelocity / 2.0));
            bottomMotor.setControl(velocityController.withVelocity(targetVelocity / 2.0));
        } else {
            //can't divide by zero
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        }
    }

    /**
     * Set the rotational velocity of the wrist joint
     * @param targetVelocity - target velocity, degrees per second
     */
    void setWristVelocity(double targetVelocity) {
        //wrist(target) velocity = top velocity - bottom velocity
        //bottom velocity = top velocity - target velocity
        //top velocity is always max unless at rest, then?
        if (targetVelocity != 0) {
            topMotor.setControl(velocityController.withVelocity(WindmillConstants.ARTICULATION_MAX_VELOCITY));
            bottomMotor.setControl(velocityController.withVelocity(WindmillConstants.ARTICULATION_MAX_VELOCITY - targetVelocity));
        } else {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        }
    }

    /**
     * Switch windmill between poses
     * @param newPose - target pose
     */
    void setPose(Pose newPose) {
        currentPose = newPose;
        setElevatorHeight(newPose.height);
        setShoulderPosition(newPose.thetaShoulder);
        setWristPosition(newPose.thetaWrist);
    }

    void stopAll() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
        elevatorL.stopMotor();
        elevatorR.stopMotor();
    }

    double getElevatorHeight() {
        return height;
    }

    //hmm
    double getShoulderPosition() {
        return 0;
    }

    double getWristPosition() {
        return 0;
    }

    //Pose enum (arbitrary poses for now)
    private enum Pose {
        HIGH(WindmillConstants.ELEVATOR_MAX_POSE, WindmillConstants.SHOULDER_MAX_POSE, WindmillConstants.WRIST_MAX_POSE),
        MIDDLE(WindmillConstants.ELEVATOR_MID_POSE, WindmillConstants.SHOULDER_MID_POSE, WindmillConstants.WRIST_MID_POSE),
        LOW(WindmillConstants.ELEVATOR_MIN_POSE, WindmillConstants.SHOULDER_MIN_POSE, WindmillConstants.WRIST_MIN_POSE);

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
