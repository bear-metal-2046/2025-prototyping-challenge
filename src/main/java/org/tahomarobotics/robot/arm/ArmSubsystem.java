package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import org.tinylog.Logger;

public class ArmSubsystem extends AbstractSubsystem {
    //Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final MotionMagicVoltage positonControl = new MotionMagicVoltage(0);


    public ArmSubsystem() {

        topMotor = new  TalonFX(RobotMap.ARM_TOP_MOTOR, RobotMap.CANBUS_NAME);

        bottomMotor =  new TalonFX(RobotMap.ARM_BOTTOM_MOTOR, RobotMap.CANBUS_NAME);

        RobustConfigurator.tryConfigureTalonFX("Arm Motor", topMotor, ArmConstants.armMotorConfig());

        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));

    }

    ArmSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
        Logger.info("Creating an instance of ArmSubsystem");
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }


    public TalonFX getTopMotor() {
        return topMotor;
    }


    public void setArmPosition(Angle angle) {
        topMotor.setControl(positonControl.withPosition(angle));

    }

    public void setArmPercentOutput(double percent){
        topMotor.setControl(new DutyCycleOut(percent));

    }

    public Angle getArmPosition(){
        return topMotor.getPosition().getValue();
    }



    @Override
    public void subsystemPeriodic() {

    }


}