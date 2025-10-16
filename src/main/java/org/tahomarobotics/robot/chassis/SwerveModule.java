package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.function.BooleanSupplier;

import static org.tahomarobotics.robot.RobotMap.CANBUS_NAME;
import static org.tahomarobotics.robot.chassis.ChassisConstants.*;


class SwerveModule extends SubsystemBase {

    //Identifiers
    private final String name;

    //Hardware
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    // state
    private SwerveModuleState targetState = new SwerveModuleState();

    //Status Signals
    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Angle> driveRotorPosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Current> driveCurrent, steerCurrent;
    private final StatusSignal<Voltage> driveVoltage, steerVoltage;


    private final LoggedStatusSignal[] statusSignals;

    //Constructors
    public SwerveModule(RobotMap.moduleId descriptor) {
        name = descriptor.moduleName();


        driveMotor = new TalonFX(descriptor.driveId(), CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.cancoderId(), CANBUS_NAME);


        RobustConfigurator.tryConfigureTalonFX(" Drive Motor", driveMotor, createDriveMotorConfig());
        RobustConfigurator.tryConfigureTalonFX(" Steer Motor", steerMotor, createSteerMotorConfig(descriptor.cancoderId(), steerRatio));
        RobustConfigurator.tryConfigureCANcoder(" Encoder ", steerEncoder, createEncoderConfig());


        statusSignals = new LoggedStatusSignal[]{
                new LoggedStatusSignal("Steer Position", steerMotor.getPosition()),
                new LoggedStatusSignal("Steer Velocity", steerMotor.getVelocity()),
                new LoggedStatusSignal("Steer Current", steerMotor.getStatorCurrent()),
                new LoggedStatusSignal("Steer Voltage", steerMotor.getMotorVoltage()),

                new LoggedStatusSignal("Drive Position", driveMotor.getPosition()),
                new LoggedStatusSignal("Drive Rotor Position", driveMotor.getRotorPosition()),
                new LoggedStatusSignal("Drive Velocity", driveMotor.getVelocity()),
                new LoggedStatusSignal("Drive Current", driveMotor.getStatorCurrent()),
                new LoggedStatusSignal("Drive Voltage", driveMotor.getMotorVoltage()),
        };

        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();
        steerCurrent = steerMotor.getStatorCurrent();
        steerVoltage = steerMotor.getMotorVoltage();


        drivePosition = driveMotor.getPosition();
        driveRotorPosition = driveMotor.getRotorPosition();
        driveVelocity = driveMotor.getVelocity();
        driveCurrent = driveMotor.getStatorCurrent();
        driveVoltage = driveMotor.getMotorVoltage();

        Logger.recordOutput("steer/position", (BooleanSupplier) steerPosition);
        Logger.recordOutput("steer/velocity", (BooleanSupplier) steerVelocity);
        Logger.recordOutput("steer/current", (BooleanSupplier) steerCurrent);
        Logger.recordOutput("steer/voltage", (BooleanSupplier) steerVoltage);

        Logger.recordOutput("drive/position", (BooleanSupplier) drivePosition);
        Logger.recordOutput("drive/rotorPosition", (BooleanSupplier) driveRotorPosition);
        Logger.recordOutput("drive/velocity", (BooleanSupplier) driveVelocity);
        Logger.recordOutput("drive/current", (BooleanSupplier) driveCurrent);
        Logger.recordOutput("drive/voltage", (BooleanSupplier) driveVoltage);





    }
}