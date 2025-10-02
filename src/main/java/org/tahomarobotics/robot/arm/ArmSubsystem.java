package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

public class ArmSubsystem extends AbstractSubsystem {
    //Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    //Absolute encoders
    private final CANcoder elbowEncoder;
    private final CANcoder wristEncoder;

    public ArmSubsystem() {
        Logger.info("Creating an instance of ArmSubsystem...");
        topMotor = new TalonFX(RobotMap.ARM_TOP_MOTOR);
        Logger.info("Arm top motor instantiated.");
        bottomMotor = new TalonFX(RobotMap.ARM_BOTTOM_MOTOR);
        Logger.info("Arm bottom motor instantiated.");
        elbowEncoder = new CANcoder(RobotMap.ELBOW_ENCODER);
        Logger.info("Arm elbow CANcoder instantiated.");
        wristEncoder = new CANcoder(RobotMap.WRIST_ENCODER);
        Logger.info("Arm wrist CANcoder instantiated.");

        RobustConfigurator.tryConfigureTalonFX("Arm Top Motor", topMotor, ArmConstants.topMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Arm Bottom Motor", bottomMotor, ArmConstants.bottomMotorConfiguration);
    }

    @Override
    public void subsystemPeriodic() {
    }
}
