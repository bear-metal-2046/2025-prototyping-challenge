package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Chassis implements AutoCloseable {
    private final ChassisSubsystem chassis;

    public Chassis() {
        this(new ChassisSubsystem());
    }

    Chassis(ChassisSubsystem chassis) {
        this.chassis = chassis;
    }


    // Teleop drive command
    public void bindTeleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
            Command teleopCommand = chassis.run(() -> {
                double forward = applyDesensitization(-x.getAsDouble(), ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
                double strafe = applyDesensitization(-y.getAsDouble(), ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
                double rot = applyDesensitization(-omega.getAsDouble(), ChassisConstants.CONTROLLER_ROTATIONAL_SENSITIVITY);
                double dir = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? -1.0 : 1.0).orElse(1.0);

                double vx = forward * ChassisConstants.MAX_SPEED * dir;
                double vy = strafe * ChassisConstants.MAX_SPEED * dir;
                double rotRate = rot * ChassisConstants.MAX_ANGULAR_RATE;
                ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotRate);
                Logger.recordOutput("ChassisSpeeds", speeds);
                chassis.setSpeeds(speeds);
            });
            chassis.setDefaultCommand(teleopCommand);
    }

    private double applyDesensitization (double value, double power) {
        value = MathUtil.applyDeadband(value, ChassisConstants.CONTROLLER_DEADBAND);
        value = value * Math.pow(value, Math.abs(power - 1));
        return value;
    }

    public void close() {}
}
