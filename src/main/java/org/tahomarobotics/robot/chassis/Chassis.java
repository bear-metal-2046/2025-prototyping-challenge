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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

public class Chassis implements AutoCloseable {

    private static final double MAX_SPEED = ChassisConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);// ChassisConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    private final ChassisSubsystem chassis;

    public Chassis() {
        this(new ChassisSubsystem());
    }

    Chassis(ChassisSubsystem chassis) {
        this.chassis = chassis;
    }

    public void bindTeleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        final var request = new SwerveRequest.FieldCentric()
                .withDeadband(MAX_SPEED * 0.1)
                .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        chassis.setDefaultCommand(
                // Drivetrain will execute this command periodically
                chassis.run(() -> chassis.setControl(request
                        .withVelocityX(-x.getAsDouble() * MAX_SPEED)
                        .withVelocityY(-y.getAsDouble() * MAX_SPEED)
                        .withRotationalRate(-omega.getAsDouble() * MAX_ANGULAR_RATE))));

    }

    public void bindTeleopDrive2(DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier omega) {
        var telopDriveCommand = chassis.run(() -> {
            double forward = applyDesensitization(-x.getAsDouble(),
                    ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
            double strafe = applyDesensitization(-y.getAsDouble(),
                    ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
            double rot = applyDesensitization(-omega.getAsDouble(),
                    ChassisConstants.CONTROLLER_ROTATIONAL_SENSITIVITY);
            double dir = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? -1.0 : 1.0).orElse(1.0);

            double vx = forward * MAX_SPEED * dir;
            double vy = strafe * MAX_SPEED * dir;
            double rotRate = rot * MAX_ANGULAR_RATE;
            ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotRate);
            chassis.setSpeeds(speeds);
        });

        chassis.setDefaultCommand(telopDriveCommand);

    }

    private double applyDesensitization(double value, double power) {
        value = MathUtil.applyDeadband(value, ChassisConstants.CONTROLLER_DEADBAND);
        value *= Math.pow(value, Math.abs(power - 1));
        return value;
    }

    public void close() {
    }

    public ChassisSimulation getSimulation() {
        return chassis.getSimulation();
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
