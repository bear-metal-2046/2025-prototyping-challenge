package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.wpilibj.CAN;

import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

public class ArmSubsystem extends AbstractSubsystem {
    //Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;


    private final ArmSimulation armSimulataion;


    ArmSubsystem() {
        this(new TalonFX(RobotMap.ARM_TOP_MOTOR), new TalonFX(RobotMap.ARM_BOTTOM_MOTOR));
    }

    ArmSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
        Logger.info("Creating an instance of ArmSubsystem (test constructor)...");
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;


        armSimulataion = new ArmSimulation(
            topMotor.getSimState(), 
            bottomMotor.getSimState()



        );
    }
    @Override
    public void subsystemPeriodic() {

    }

    public ArmSimulation getSimulation() {
        return armSimulataion;
    }
}
