package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.ELEVATOR_MAIN_PULLEY_RADIUS;

public class ElevatorSimulation extends AbstractSimulation {

    public ElevatorSimulation(TalonFXSimState elevatorMotorSimState, CANcoderSimState elevatorEncoderSimState) {
        super("Elevator");
        motorSimState = elevatorMotorSimState;
        encoderSimState = elevatorEncoderSimState;
        sim = new ElevatorSim(
                ELEV_MOTOR,
                52d/12d, // Gear Ratio
                ELEV_ARM_MASS.in(Kilograms),
                ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters),
                MIN_HEIGHT.in(Meters),
                MAX_HEIGHT.in(Meters),
                true,
                START_HEIGHT.in(Meters)
        );
    }




    public Distance getPosition() {
        return position;
    }

    public ElevatorSim getElevatorSim() {
        return sim;
    }

    @Override
    public void simulationPeriodic() {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        double voltage = motorSimState.getMotorVoltage();

        sim.setInputVoltage(voltage);
        sim.update(Robot.defaultPeriodSecs);

        Angle encoderAngle = Radians.of(sim.getPositionMeters() / ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters));
        AngularVelocity encoderVelocity = RadiansPerSecond.of(sim.getVelocityMetersPerSecond()/ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters));

        motorSimState.setRawRotorPosition(encoderAngle);
        motorSimState.setRotorVelocity(encoderVelocity);
        encoderSimState.setRawPosition(encoderAngle);
        encoderSimState.setVelocity(encoderVelocity);
        Logger.recordOutput("Elevator/SimPosition", sim.getPositionMeters());
        Logger.recordOutput("Elevator/SimVelocity", sim.getVelocityMetersPerSecond());
        Logger.recordOutput("Elevator/InputVoltage", voltage);
        position = Meters.of(sim.getPositionMeters());
    }

    private static final DCMotor ELEV_MOTOR = DCMotor.getKrakenX60(2);
    private static final Distance MIN_HEIGHT = Meters.of(0.0);
    private static final Distance MAX_HEIGHT = Meters.of(1.2);
    private static final Distance START_HEIGHT = Meters.of(0.0);
    private static final Mass ELEV_ARM_MASS = Pounds.of(25.0);

    private final TalonFXSimState motorSimState;
    private final CANcoderSimState encoderSimState;
    private final ElevatorSim sim;

    private Distance position = Meters.zero();

}
