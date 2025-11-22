package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.tahomarobotics.robot.Robot;
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
    private final CANcoder encoder;

    private final ElevatorSimulation simulation; // may be null when not in simulation

    //Variables
    private Distance targetPos = ELEVATOR_MIN_POSE;

    //Status Signals
    private final StatusSignal<Angle> carriagePos;
    private final StatusSignal<AngularVelocity> carriageVelocity;

    //Control request

    private final PositionVoltage elevatorControlRequest = new PositionVoltage(0);

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

        // Create a persistent CANcoder instance (avoid creating a temporary just for sim state)
        encoder = new CANcoder(RobotMap.ELEVATOR_ENCODER);

        // Only construct the simulation helper when running in simulation
        if (Robot.isSimulation()) {
            simulation = new ElevatorSimulation(leftMotor.getSimState(), encoder.getSimState());
        } else {
            simulation = null;
        }
    }

    public ElevatorSimulation getSimulation() {
        return simulation;
    }

    //Set Control

    //Set the Voltage of the elevator left motor
    public void setElevatorVoltage(Voltage volts) {
        leftMotor.setVoltage(volts.in(Volt));
    }

    //Moves the target Position, with set limits
    public void moveToPosition(Distance position) {
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

    //Get Values
    public AngularVelocity getCarriageVelocity() {
        return carriageVelocity.getValue();
    }

    public void setCarriagePos(Distance pos) {
        leftMotor.setPosition(pos.in(Meters));
    }

    //Periodic
    @Override
    public void subsystemPeriodic() {
        //Logging State and Position Data
        Logger.recordOutput("Elevator/Target Position", targetPos);
        Logger.recordOutput("Elevator/Position error", (targetPos.in(Meters)
                - (carriagePos.getValue().in(Rotations)
                * ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Meters))));
        Logger.recordOutput("Elevator/Carriage/Carriage position", carriagePos.getValue());
        Logger.recordOutput("Elevator/Carriage/Carriage velocity", carriageVelocity.getValue());
        Logger.recordOutput("Elevator/Carriage/Target Position", targetPos);
        Logger.recordOutput("Elevator/Carriage/Carriage position", carriagePos.getValue());
        Logger.recordOutput("Elevator/Carriage/Carriage velocity", carriageVelocity.getValue());
        Logger.recordOutput("Elevator/Carriage/Position error", (targetPos.in(Meters)
                - (carriagePos.getValue().in(Rotations)
                * ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE.in(Meters))));

        //Logging Motor and Control Data
        Logger.recordOutput("Elevator/Voltages/Left Motor", leftMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Voltages/Right Motor", rightMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Stator Current", leftMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Left Motor Supply Current", leftMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Stator Current", rightMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Currents/Right Motor Supply Current", rightMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Temperature/Right Motor Temperature", rightMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/Temperature/Left Motor Temperature", leftMotor.getDeviceTemp().getValue());

        //Logging Raw Sensor Feedback
        Logger.recordOutput("Elevator/Sensor Feedback/Left Motor", leftMotor.getPosition().getValue());
        Logger.recordOutput("Elevator/Sensor Feedback/Right Motor", rightMotor.getPosition().getValue());

        //Logging limit switches
        Logger.recordOutput("Elevator/Limit Switches/Max Limit", leftMotor.getFault_ForwardSoftLimit().getValue());
        Logger.recordOutput("Elevator/Limit Switches/Min Limit", leftMotor.getFault_ReverseSoftLimit().getValue());
    }


    public void updateTelemetry() {
        if (simulation != null) {
            simulation.updateTelemetry();
        }
    }

    public boolean isAtLowerLimit() {
        return simulation != null && simulation.isAtLowerLimit();
    }

    public boolean isAtUpperLimit() {
        return simulation != null && simulation.isAtUpperLimit();
    }


}