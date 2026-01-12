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

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.vision.VisionConstants;
import org.tahomarobotics.robot.vision.Limelight.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

public class Chassis implements AutoCloseable {

    private final ChassisSubsystem chassis;

    public Chassis() {
        this(new ChassisSubsystem());
    }

    Chassis(ChassisSubsystem chassis) {
        this.chassis = chassis;
    }

    public void bindTeleopDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        final var maxSpeed = ChassisConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
        final var maxRotateRate = ChassisConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

        final var request = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxRotateRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        chassis.setDefaultCommand(
                chassis.run(() -> chassis.setControl(request
                        .withVelocityX(-forward.getAsDouble() * maxSpeed)
                        .withVelocityY(-strafe.getAsDouble() * maxSpeed)
                        .withRotationalRate(-rotate.getAsDouble() * maxRotateRate))));

    }

    public Command alignSwerves(Trigger complete) {
        return new FunctionalCommand(
                chassis::initAlign,
                () -> {},
                chassis.finishAlign(),
                complete,
                chassis)
            .ignoringDisable(true);
    }

    public void close() {
        chassis.close();
    }


    public ChassisSimulation getSimulation() {
        return chassis.getSimulation();
    }

    public void addVisionPositionMeasurement(Pose2d pose, double timestampSeconds, Vector<N3> stdDevs) {
        chassis.setVisionMeasurementStdDevs(stdDevs);
        // Time in Networktables (where the position estimation time comes from) is recorded in FPGA time, 
        // while the CTRE Swerve pose estimator uses current time, so we have to convert to current time here. 
        // See https://www.chiefdelphi.com/t/ctre-swerve-addvisionmeasurment-having-no-effect/482024/5
        chassis.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Pose2d getPose() {
        return chassis.getStateCopy().Pose;
    }

    /**
     * returns a reset pose command where an auto can initialize it position at the
     * starting point on the field
     */
    public Command resetRobotPose(Pose2d pose) {
        return Commands.runOnce(() -> chassis.setPose(pose))
                .ignoringDisable(true)
                .withName("chassis::resetRobotPose");
    }
}
