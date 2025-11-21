package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class Chassis implements AutoCloseable {
    private final ChassisSubsystem chassis;

    private double MaxSpeed = ChassisConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    private double MaxAngularRate = ChassisConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    public Chassis() {
            this(new ChassisSubsystem());
        }

    Chassis(ChassisSubsystem chassis){this.chassis = chassis;}

    // Teleop drive command
    public Command teleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        return null;
    }

    public double applyDesensitization (double value, double power) {
        value = MathUtil.applyDeadband(value, ChassisConstants.CONTROLLER_DEADBAND);
        value = value * Math.pow(value, Math.abs(power - 1));
        return value;
    }

    public void close() {}
}
