package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

public class Arm implements AutoCloseable {
    private final ArmSubsystem arm;


    private ClimberState armState = ClimberState.LOW;

    public enum ClimberState {
        LOW, MID, HIGH, STOW
    }

    public Arm() {
        this(new ArmSubsystem());
        Logger.info("Creating an instance of Arm...");
    }

    Arm(ArmSubsystem arm) {
        this.arm = arm;

    }

    public Command sysIdQuasistaticForward() {
        return ArmSubsystem.getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return ArmSubsystem.getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return ArmSubsystem.getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return ArmSubsystem.getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse);
    }


    public Command lowCommand(){
    armState = ClimberState.LOW;

    return arm.runOnce(() -> arm.setArmPosition(ArmConstants.LOW));


    }

    public  Command midCommand(){

        armState = ClimberState.MID;

        return arm.runOnce(() -> arm.setArmPosition(ArmConstants.MID));

    }

    public  Command highCommand(){

        armState = ClimberState.HIGH;

        return arm.runOnce(() -> arm.setArmPosition(ArmConstants.HIGH));

    }

    public  Command stowCommand(){

        armState = ClimberState.STOW;

        return arm.runOnce(() -> arm.setArmPosition(ArmConstants.STOW));

    }

    public Command stopCommand(){
        return arm.runOnce(() -> arm.getTopMotor().stopMotor());

    }




    @Override
    public void close() throws Exception {

    }

}
