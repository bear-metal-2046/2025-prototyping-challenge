package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
     private static SysIdRoutine sysIdRoutine;

    //Status Signals
     private StatusSignal <Angle> carriagePos;
     private StatusSignal <AngularVelocity> carriageVelocity;

     //Control request

    private final PositionVoltage elevatorControlRequest = new PositionVoltage(0);

    //Constructor
    public ElevatorSubsystem() {

        //Motors
        leftMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_LEFT);
        rightMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_RIGHT);
        RobustConfigurator.tryConfigureTalonFX("left motor", leftMotor, elevatorMotorConfig);
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_MOTOR_LEFT, true));

        //Status Signals
        carriagePos = leftMotor.getPosition();
        carriageVelocity = leftMotor.getVelocity();

        // SysID
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(

                        (Voltage volts) -> {
                            double battery = RobotController.getBatteryVoltage();
                            double percent = battery > 0.0 ? volts.in(Volt) / battery : 0.0;
                            leftMotor.setControl(new DutyCycleOut(percent));
                        },
                        (state) -> SignalLogger.writeString("motorState", state.toString()),
                        this
                )
        );

        //Logging Elevator Creation
        org.tinylog.Logger.info("Creating ElevatorSubsystem");
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

    public void setCarriagePos(Distance pos) {
        leftMotor.setPosition(pos.in(Meters));
    }

    //Stops both left and right motors
    public void stop(){
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

    //Get Values
    public AngularVelocity getCarriageVelocity () {
        return carriageVelocity.getValue();
    }

    //SysID
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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
        Logger.recordOutput("Elevator/Temperature/Right Motor Temperature", rightMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/Temperature/Left Motor Temperature", leftMotor.getDeviceTemp().getValue());

        //Logging Raw Sensor Feedback
        Logger.recordOutput("Elevator/Sensor Feedback/Left Motor", leftMotor.getPosition().getValue());
        Logger.recordOutput("Elevator/Sensor Feedback/Right Motor", rightMotor.getPosition().getValue());

        //Logging limit switches
        Logger.recordOutput("Elevator/Limit Switches/Max Limit", leftMotor.getFault_ForwardSoftLimit().getValue());
        Logger.recordOutput("Elevator/Limit Switches/Min Limit", leftMotor.getFault_ReverseSoftLimit().getValue());
    }


}