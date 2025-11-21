package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.ELEVATOR_MAIN_PULLEY_RADIUS;

public final class ElevatorSimulation extends AbstractSimulation {

    public ElevatorSimulation(TalonFXSimState elevatorMotorSimState, CANcoderSimState elevatorEncoderSimState) {
        super("Elevator");
        motorSimState = elevatorMotorSimState;
        encoderSimState = elevatorEncoderSimState;


        SmartDashboard.putData("ElevatorSim", mech2d);
        SmartDashboard.putData("Field", field);
    }

    private Field2d field = new Field2d();
    final ElevatorSim sim = new ElevatorSim(
            ELEV_MOTOR,
            52d/12d, // Gear Ratio
            ELEV_ARM_MASS.in(Kilograms),
            ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters),
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            true,
            START_HEIGHT.in(Meters));



    @Override
    public void simulationPeriodic() {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        double voltage = motorSimState.getMotorVoltage();

        // apply requested voltage
        sim.setInputVoltage(voltage);
        // advance simulation
        sim.update(Robot.defaultPeriodSecs);


        // read sim results
        double pos = sim.getPositionMeters();
        double vel = sim.getVelocityMetersPerSecond();

        // determine limit switch states (bottom = lower limit)
        final double eps = 1e-6;
        atLowerLimit = pos <= MIN_HEIGHT.in(Meters) + eps;
        atUpperLimit = pos >= MAX_HEIGHT.in(Meters) - eps;

        if ((atLowerLimit && voltage < 0.0) || (atUpperLimit && voltage > 0.0)) {
            // stop applying further voltage
            sim.setInputVoltage(0.0);
            // clamp position/velocity to the limit so sensors reflect the stop
            double clampedPos = Math.max(MIN_HEIGHT.in(Meters), Math.min(MAX_HEIGHT.in(Meters), pos));
            // update encoder/motor sim state to the clamped position
            Angle encoderAngle = Radians.of(clampedPos / ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters));
            motorSimState.setRawRotorPosition(encoderAngle);
            motorSimState.setRotorVelocity(RadiansPerSecond.of(0.0));
            encoderSimState.setRawPosition(encoderAngle);
            encoderSimState.setVelocity(RadiansPerSecond.of(0.0));
            // publish clamped position as telemetry
            Logger.recordOutput("Elevator/Sim/Position", clampedPos);
            Logger.recordOutput("Elevator/Sim/Velocity", 0.0);
        } else {
            Angle encoderAngle = Radians.of(pos / ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters));
            AngularVelocity encoderVelocity = RadiansPerSecond.of(vel / ELEVATOR_MAIN_PULLEY_RADIUS.in(Meters));

            motorSimState.setRawRotorPosition(encoderAngle);
            motorSimState.setRotorVelocity(encoderVelocity);
            encoderSimState.setRawPosition(encoderAngle);
            encoderSimState.setVelocity(encoderVelocity);

            Logger.recordOutput("Elevator/Sim/Position", pos);
            Logger.recordOutput("Elevator/Sim/Velocity", vel);
        }

        // Simple estimated motor current for telemetry (not a physics-accurate measurement)
        // Scale: 0..12V -> 0..estMaxCurrentA
        final double estMaxCurrentA = 120.0; // conservative estimate for a paired KrakenX60 setup
        double estimatedCurrent = Math.abs(voltage) / 12.0 * estMaxCurrentA;
        Logger.recordOutput("Elevator/Sim/EstimatedCurrent", estimatedCurrent);

        // publish limit flags
        Logger.recordOutput("Elevator/Sim/Limit/Lower", atLowerLimit);
        Logger.recordOutput("Elevator/Sim/Limit/Upper", atUpperLimit);
    }
    private static final DCMotor ELEV_MOTOR = DCMotor.getKrakenX60(2);
    private static final Distance MIN_HEIGHT = Meters.of(0.0);
    private static final Distance MAX_HEIGHT = Meters.of(1.2);
    private static final Distance START_HEIGHT = Meters.of(0.0);
    private static final Mass ELEV_ARM_MASS = Pounds.of(25.0);

    private final TalonFXSimState motorSimState;
    private final CANcoderSimState encoderSimState;


    private boolean atLowerLimit = false;
    private boolean atUpperLimit = false;

    private final Mechanism2d mech2d =
            new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(51));
    private final MechanismRoot2d mech2dRoot =
            mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
    private final MechanismLigament2d elevatorMech2d =
            mech2dRoot.append(
                    new MechanismLigament2d("Elevator", sim.getPositionMeters(), 90));


    public void updateTelemetry() {
        elevatorMech2d.setLength(sim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Position", sim.getPositionMeters());
    }

    // Accessors used by subsystem/tests
    public boolean isAtLowerLimit() {
        return atLowerLimit;
    }

    public boolean isAtUpperLimit() {
        return atUpperLimit;
    }

    public double getSimPositionMeters() {
        return sim.getPositionMeters();
    }

}
