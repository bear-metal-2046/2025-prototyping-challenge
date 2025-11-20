package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
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



    public Command setPosition(double angleDegrees) {
        return arm.runOnce(() -> {

            double motorRotations = (angleDegrees / 360.0);
            arm.getTopMotor().setControl(new MotionMagicVoltage(motorRotations));
        });
    }


    public Command lowCommand(){
    armState = ClimberState.LOW;

    return arm.runOnce(() -> arm.getTopMotor().setPosition(ArmConstants.LOW));


    }

    public  Command midCommand(){

        armState = ClimberState.MID;

        return arm.runOnce(() -> arm.getTopMotor().setPosition(ArmConstants.MID));

    }

    public  Command highCommand(){

        armState = ClimberState.HIGH;

        return arm.runOnce(() -> arm.getTopMotor().setPosition(ArmConstants.HIGH));

    }

    public  Command stowCommand(){

        armState = ClimberState.STOW;

        return arm.runOnce(() -> arm.getTopMotor().setPosition(ArmConstants.STOW));

    }

    public Command stopCommand(){

        return arm.runOnce(() -> arm.getTopMotor().stopMotor());

    }

    public Command setPercentOutput(double percent){

        return arm.runOnce(() -> arm.getTopMotor().setControl(new DutyCycleOut(percent)));
    }

    public Command getPosition(){

        return arm.runOnce(() ->  arm.getTopMotor().getPosition().getValue());
    }



    @Override
    public void close() throws Exception {

    }

}
