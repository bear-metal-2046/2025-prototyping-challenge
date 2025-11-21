package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import org.tinylog.Logger;

import static edu.wpi.first.units.Units.Volt;

public class ArmSubsystem extends AbstractSubsystem {
    //Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final MotionMagicVoltage positonControl = new MotionMagicVoltage(0);

    // SysId routine for arm characterization
    private static SysIdRoutine sysIdRoutine;


    public ArmSubsystem() {

        topMotor = new  TalonFX(RobotMap.ARM_TOP_MOTOR, RobotMap.CANBUS_NAME);

        bottomMotor =  new TalonFX(RobotMap.ARM_BOTTOM_MOTOR, RobotMap.CANBUS_NAME);

        RobustConfigurator.tryConfigureTalonFX("Arm Motor", topMotor, ArmConstants.armMotorConfig());

        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(

                (Voltage volts) -> {
                   double battery = RobotController.getBatteryVoltage();
                    double percent = battery > 0.0 ? volts.in(Volt) / battery : 0.0;
                    topMotor.setControl(new DutyCycleOut(percent));
                },
                log -> {},
                this
            )
        );

    }

    ArmSubsystem(TalonFX topMotor, TalonFX bottomMotor, SysIdRoutine sysIdRoutine) {
        this.sysIdRoutine = sysIdRoutine;
        Logger.info("Creating an instance of ArmSubsystem");
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;


    }

    public static SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
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

    // Expose SysId commands for use (e.g., bound to joystick buttons)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }



    @Override
    public void subsystemPeriodic() {

    }


}