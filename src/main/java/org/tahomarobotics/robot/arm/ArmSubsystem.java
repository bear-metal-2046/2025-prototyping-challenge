package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.wpilibj.CAN;

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
    private final CANcoder wristEncoder;

    private final ArmSimulation armSimulataion;


    ArmSubsystem() {
        Logger.info("Creating an instance of ArmSubsystem...");
        topMotor = new TalonFX(RobotMap.ARM_TOP_MOTOR);
        bottomMotor = new TalonFX(RobotMap.ARM_BOTTOM_MOTOR);
        topEncoder = new CANcoder(RobotMap.ARM_TOP_ENCODER);
        bottomEncoder = new CANcoder(RobotMap.ARM_BOTTOM_ENCODER);


        RobustConfigurator.tryConfigureTalonFX("Arm Top Motor", topMotor, ArmConstants.topMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Arm Bottom Motor", bottomMotor, ArmConstants.bottomMotorConfiguration);
        this(new TalonFX(RobotMap.ARM_TOP_MOTOR), new TalonFX(RobotMap.ARM_BOTTOM_MOTOR),
            new CANcoder(RobotMap.ARM_TOP_ENCODER),   new CANcoder(RobotMap.ARM_BOTTOM_ENCODER),
            new CANcoder(RobotMap.ARM_WRIST_ENCODER));
    }

    ArmSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANcoder topEncoder, CANcoder bottomEncoder, CANcoder wristEncoder) {
        Logger.info("Creating an instance of ArmSubsystem (test constructor)...");
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.topEncoder = topEncoder;
        this.bottomEncoder = bottomEncoder;
        this.wristEncoder = wristEncoder;

        armSimulataion = new ArmSimulation(
            topMotor.getSimState(), 
            bottomMotor.getSimState(), 
            topEncoder.getSimState(),
            bottomEncoder.getSimState(),
            wristEncoder.getSimState()
        );
    }
    @Override
    public void subsystemPeriodic() {
    }

    public ArmSimulation getSimulation() {
        return armSimulataion;
    }
}
