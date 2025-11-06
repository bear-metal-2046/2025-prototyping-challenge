package org.tahomarobotics.robot.chassis;

import org.tahomarobotics.robot.sim.AbstractSimulation;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;


public class ChassisSimulation extends AbstractSimulation {
    record SwerveModuleSimState(TalonFXSimState driveMotorState, TalonFXSimState steerMotorState, CANcoderSimState steerEncoderState) {}

    SwerveDriveSimulation swerveDriveSimulation;

    /**
     * Constructs a ChassisSimulation instance.
     *
     * @param moduleSimStates An array of SwerveModuleSimState objects representing the simulation states
     *                        of the swerve modules. The array should be ordered as follows:
     *                        front left, front right, back left, back right.
     */
    public ChassisSimulation(SwerveModuleSimState[] moduleSimStates) {
        super("ChassisSimulation");

        swerveDriveSimulation = new SwerveDriveSimulation(ChassisConstants.swerveDriveConfig(), new Pose2d());

    }

    @Override
    protected void simulationPeriodic() {
    }
}