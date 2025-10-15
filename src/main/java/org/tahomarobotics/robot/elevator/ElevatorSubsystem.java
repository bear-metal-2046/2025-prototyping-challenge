package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

class ElevatorSubsystem extends AbstractSubsystem {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public ElevatorSubsystem() {
        Logger.info("Creating ElevatorSubsystem");
        leftMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
        rightMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);
        RobustConfigurator.tryConfigureTalonFX("left motor", leftMotor, ElevatorConstants.elevatorMotorConfig);
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_MOTOR_LEFT, true));
    }

    @Override
    public void subsystemPeriodic() {}

    public void moveToPosition(double position) {
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }
}