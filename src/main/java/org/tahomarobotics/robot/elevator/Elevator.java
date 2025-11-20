package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import org.tinylog.Logger;

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

    @Override
    public void close() throws Exception {}
}
