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
    private final CANcoder topEncoder;
    private final CANcoder bottomEncoder;

    public ArmSubsystem() {
        Logger.info("Creating an instance of ArmSubsystem...");
        topMotor = new TalonFX(RobotMap.ARM_TOP_MOTOR);
        Logger.info("Arm top motor instantiated.");
        bottomMotor = new TalonFX(RobotMap.ARM_BOTTOM_MOTOR);
        Logger.info("Arm bottom motor instantiated.");
        topEncoder = new CANcoder(RobotMap.TOP_ENCODER);
        Logger.info("Arm top encoder instantiated.");
        bottomEncoder = new CANcoder(RobotMap.BOTTOM_ENCODER);
        Logger.info("Arm bottom encoder instantiated.");


        RobustConfigurator.tryConfigureTalonFX("Arm Top Motor", topMotor, ArmConstants.topMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Arm Bottom Motor", bottomMotor, ArmConstants.bottomMotorConfiguration);
    }

    @Override
    public void subsystemPeriodic() {
    }
}
