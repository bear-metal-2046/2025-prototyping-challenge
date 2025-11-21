package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tinylog.Logger;
import org.tinylog.Supplier;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.ELEVATOR_ZERO_VOLTAGE;

public class Elevator implements AutoCloseable {
    private final ElevatorSubsystem elevator;



    public Elevator() {
        this(new ElevatorSubsystem());
    }

    public Elevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevator = elevatorSubsystem;
        Logger.info("Elevator initialized");
    }

    // Basic commands
    public Command setPosition(Distance position) {
        return elevator.runOnce(() -> elevator.moveToPosition(position))
                .withName("Elevator move to Position");
    }
    public Command stop() {
        return elevator.runOnce(() -> elevator.stop())
                .withName("Elevator Stop");
    }

    public Command homing() {
        return elevator.runOnce(() -> elevator.setElevatorVoltage(ELEVATOR_ZERO_VOLTAGE.in(Volt)))
                .andThen(Commands.waitUntil(() -> elevator.getCarriageVelocity().isNear(Rotations.per(Second).of(0.0), Rotations.per(Second).of(0.01))))
                .andThen(Commands.runOnce(() -> elevator.setCarriagePos(0.0)))
                .andThen(Commands.runOnce(() -> Logger.info("Elevator has Homed")));
    }

    public Command manualJog(Supplier<Voltage> voltage) {
        return Commands.run(() ->elevator.setElevatorVoltage(voltage.get().in(Volt)))
                .andThen(Commands.runOnce(() -> Logger.info("Elevator is Jogging")));
    }

    @Override
    public void close() throws Exception {}
}
