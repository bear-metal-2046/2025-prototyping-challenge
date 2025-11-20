package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

class ElevatorSubsystem extends AbstractSubsystem {

    //hardware
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    //Variables
     private Distance targetPos = ELEVATOR_MIN_POSE;

    //Status Signals
     private StatusSignal <Angle> carriagePos;
     private StatusSignal <AngularVelocity> carriageVelocity;

    //Constructor
    public ElevatorSubsystem() {

        //Motors
        leftMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
        rightMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);
        RobustConfigurator.tryConfigureTalonFX("left motor", leftMotor, ElevatorConstants.elevatorMotorConfig);
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_MOTOR_LEFT, true));

        //Status Signals
        carriagePos = leftMotor.getPosition();
        carriageVelocity = leftMotor.getVelocity();

        //Logging Elevator Creation
        org.tinylog.Logger.info("Creating ElevatorSubsystem");
    }

    //Set Control

    //Set the Voltage of the elevator left motor
    public void setElevatorVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    //Currently does not move anything, and only sets the target position
    public void moveToPosition(Distance position, ControlRequest elevatorControlRequest) {
        org.tinylog.Logger.info(position);
        targetPos = Meters.of(MathUtil.clamp(position.in(Meters), ELEVATOR_MIN_POSE.in(Meters), ELEVATOR_MAX_POSE.in(Meters)));
        leftMotor.setControl(elevatorControlRequest);
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_MOTOR_LEFT, true));
    }

    //Stops both left and right motors
    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    //Periodic
    @Override
    public void subsystemPeriodic() {
        //Logging State and Position Data
        Logger.recordOutput("Elevator/Carriage/Target Position", targetPos);
        Logger.recordOutput("Elevator/Carriage/Carriage position", carriagePos.getValue());
        Logger.recordOutput("Elevator/Carriage/Carriage velocity", carriageVelocity.getValue());
        Logger.recordOutput("Elevator/Carriage/Position error", (targetPos.in(Meters)
                -(carriagePos.getValue().in(Rotations)
                *ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Meters))));

        //Logging Motor and Control Data
        Logger.recordOutput("Elevator/Voltages/Left Motor", leftMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Voltages/Right Motor", rightMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Stator Current", leftMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Supply Current", leftMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Stator Current", rightMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Supply Current", rightMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Temperature/Right Motor Supply Current", rightMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/Temperature/Left Motor Supply Current", leftMotor.getDeviceTemp().getValue());

        //Logging Raw Sensor Feedback
        Logger.recordOutput("Elevator/Sensor Feedback/Left Motor", leftMotor.getPosition().getValue());
        Logger.recordOutput("Elevator/Sensor Feedback/Right Motor", rightMotor.getPosition().getValue());

        //Logging limit switches
        Logger.recordOutput("Elevator/Limit Switches/Max Limit", leftMotor.getFault_ForwardSoftLimit().getValue());
        Logger.recordOutput("Elevator/Limit Switches/Min Limit", leftMotor.getFault_ReverseSoftLimit().getValue());
    }


}