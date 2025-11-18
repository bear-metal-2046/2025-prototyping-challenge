package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE;

class ElevatorSubsystem extends AbstractSubsystem {

    //hardware
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    //Varables
     private Distance targetPos;
     private StatusSignal <Angle> carriagePos;
     private StatusSignal <AngularVelocity> carriageVelocity;

    //Status Signals
    public ElevatorSubsystem() {
        org.tinylog.Logger.info("Creating ElevatorSubsystem");
        leftMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
        rightMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);
        RobustConfigurator.tryConfigureTalonFX("left motor", leftMotor, ElevatorConstants.elevatorMotorConfig);
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_MOTOR_LEFT, true));

        carriagePos = leftMotor.getPosition();
        carriageVelocity = leftMotor.getVelocity();

    }

    @Override
    public void subsystemPeriodic() {
        //Logging State and Position Data
        Logger.recordOutput("Elevator/Target Position", targetPos);
        Logger.recordOutput("Elevator/Carriage position", carriagePos.getValue());
        Logger.recordOutput("Elevator/Position error", (targetPos.in(Meters)
                -(carriagePos.getValue().in(Rotations)
                    *ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Meters))));
        Logger.recordOutput("Elevator/Carriage velocity", carriageVelocity.getValue());

        //Logging Motor and Control Data
        Logger.recordOutput("Elevator/Voltages/Left Motor", leftMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Stator Current", leftMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Supply Current", leftMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Temperature/Left Motor Supply Current", leftMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/Voltages/Right Motor", rightMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Stator Current", rightMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Supply Current", rightMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Temperature/Right Motor Supply Current", rightMotor.getDeviceTemp().getValue());

        //Logging Raw Sensor Feedback
        Logger.recordOutput("Elevator/Sensor Feedback/Left Motor", leftMotor.getPosition().getValue());
        Logger.recordOutput("Elevator/Sensor Feedback/Right Motor", rightMotor.getPosition().getValue());
    }

    public void moveToPosition(Distance position) {
        targetPos = position;
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }
}