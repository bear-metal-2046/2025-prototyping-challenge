package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.run;

public class Chassis implements AutoCloseable {
    private final ChassisSubsystem chassis;

    private final double MaxSpeed = ChassisConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    private final double MaxAngularRate = ChassisConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    public Chassis() {
        this(new ChassisSubsystem());
    }

    Chassis(ChassisSubsystem chassis) {
        this.chassis = chassis;
    }


    // Teleop drive command
    public Command teleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {

            double forward = applyDesensitization(-x.getAsDouble(), ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
            double strafe = applyDesensitization(y.getAsDouble(), ChassisConstants.CONTROLLER_TRANSLATIONAL_SENSITIVITY);
            double rot = applyDesensitization(omega.getAsDouble(), ChassisConstants.CONTROLLER_ROTATIONAL_SENSITIVITY);
            double dir = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red ? -1.0 : 1.0).orElse(1.0);

            double vx = forward * MaxSpeed * dir;
            double vy = strafe * MaxSpeed * dir;
            double rotRate = rot * MaxAngularRate;
            chassis.setSpeeds(new ChassisSpeeds(vx, vy, rotRate));;
            return Commands.run(() -> chassis.setSpeeds(new ChassisSpeeds(vx, vy, rotRate)));
    }

    public double applyDesensitization (double value, double power) {
        value = MathUtil.applyDeadband(value, ChassisConstants.CONTROLLER_DEADBAND);
        value = value * Math.pow(value, Math.abs(power - 1));
        return value;
    }

    public void setDefaultCommand(Command command) {
        chassis.setDefaultCommand(command);
    }

    public void close() {}
}
