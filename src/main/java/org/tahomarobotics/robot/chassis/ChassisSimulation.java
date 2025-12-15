/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.tahomarobotics.robot.chassis;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.tahomarobotics.robot.sim.Arena2025Reefscape;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

public class ChassisSimulation extends AbstractSimulation{

    private final ChassisReference DRIVE_MOTOR_ORIENTATION = ChassisReference.Clockwise_Positive;
    private final ChassisReference STEER_MOTOR_ORIENTATION = ChassisReference.Clockwise_Positive;
    private final ChassisReference STEER_ENCODER_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    private final Angle MAGNETIC_OFFSET = Degrees.zero();

    private final Pigeon2SimState gyroSimState;
    private final SwerveDriveSimulation driveSimulation;
    private final GyroSimulation gyroSimulation;

    public ChassisSimulation(Pigeon2 pigeon2, SwerveModule<TalonFX,TalonFX,CANcoder>[] modules) {
        super("Chassis Simulation");

        driveSimulation = new SwerveDriveSimulation(ChassisConstants.driveTrainSimulationConfig(), new Pose2d());
        Arena2025Reefscape.getInstance().setSwerveDriveSimulation(driveSimulation);

        SwerveModuleSimulation[] moduleSimulations = driveSimulation.getModules();
        for (int i = 0; i < moduleSimulations.length; i++) {
            moduleSimulations[i].useDriveMotorController(getDriveSimMotorController(
                modules[i].getDriveMotor().getSimState())
            );
            moduleSimulations[i].useSteerMotorController(getSteerSimMotorController(
                modules[i].getSteerMotor().getSimState(),
                modules[i].getEncoder().getSimState())
            );
        }

        gyroSimState = pigeon2.getSimState();
        gyroSimulation = driveSimulation.getGyroSimulation();
    }

        /**
     * return the MapleSim controller for drive motors
     */
    private SimulatedMotorController getDriveSimMotorController(TalonFXSimState driveMotorSimState) {
        driveMotorSimState.Orientation = DRIVE_MOTOR_ORIENTATION;
        return (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> {
            driveMotorSimState.setRawRotorPosition(encoderAngle);
            driveMotorSimState.setRotorVelocity(encoderVelocity);
            driveMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return driveMotorSimState.getMotorVoltageMeasure();
        };
    }

    /**
     * return the MapleSim controller for steer motors and encoders
     */
    private SimulatedMotorController getSteerSimMotorController(TalonFXSimState steerMotorSimState, CANcoderSimState steerEncoderSimState) {
        steerMotorSimState.Orientation = STEER_MOTOR_ORIENTATION;
        steerEncoderSimState.Orientation = STEER_ENCODER_ORIENTATION;
        return (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> {
            steerEncoderSimState.setRawPosition(mechanismAngle.minus(MAGNETIC_OFFSET));
            steerEncoderSimState.setVelocity(mechanismVelocity);
            steerMotorSimState.setRawRotorPosition(encoderAngle);
            steerMotorSimState.setRotorVelocity(encoderVelocity);
            steerMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return steerMotorSimState.getMotorVoltageMeasure();
        };
    }
    @Override
    public void simulationPeriodic() {
        
        // update gyro angle
        gyroSimState.setRawYaw(gyroSimulation.getGyroReading().getDegrees());
        gyroSimState.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity());

        Logger.recordOutput("Sim/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    }

}
