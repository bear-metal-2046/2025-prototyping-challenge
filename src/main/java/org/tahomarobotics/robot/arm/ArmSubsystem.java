package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;


import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import org.tinylog.Logger;

public class ArmSubsystem extends AbstractSubsystem {
    //Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;


    public ArmSubsystem() {

        topMotor = new TalonFX(RobotMap.ARM_TOP_MOTOR, RobotMap.CANBUS_NAME);

        bottomMotor = new TalonFX(RobotMap.ARM_BOTTOM_MOTOR, RobotMap.CANBUS_NAME);

        RobustConfigurator.tryConfigureTalonFX("Arm Motor", topMotor, ArmConstants.armMotorConfig());

        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));

    }

    ArmSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
        Logger.info("Creating an instance of ArmSubsystem");
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;


    }

    @Override
    public void subsystemPeriodic() {

    }


}