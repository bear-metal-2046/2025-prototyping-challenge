package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;

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
    private final StatusSignal<Current> driveStatorCurrent, steerStatorCurrent;
    private final StatusSignal<Current> driveSupplyCurrent, steerSupplyCurrent;
    private final StatusSignal<Voltage> driveVoltage, steerVoltage;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

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


        //steer
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();
        steerStatorCurrent = steerMotor.getStatorCurrent();
        steerVoltage = steerMotor.getMotorVoltage();
        steerSupplyCurrent = steerMotor.getSupplyCurrent();

        //drive
        drivePosition = driveMotor.getPosition();
        driveRotorPosition = driveMotor.getRotorPosition();
        driveVelocity = driveMotor.getVelocity();
        driveStatorCurrent = driveMotor.getStatorCurrent();
        driveVoltage = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();

        //encoder
        encoderPosition = steerEncoder.getPosition();
        encoderVelocity = steerEncoder.getVelocity();



        statusSignals = new LoggedStatusSignal[]{
                new LoggedStatusSignal("Steer Position", steerMotor.getPosition()),
                new LoggedStatusSignal("Steer Velocity", steerMotor.getVelocity()),
                new LoggedStatusSignal("Steer Stator Current", steerMotor.getStatorCurrent()),
                new LoggedStatusSignal("Steer Voltage", steerMotor.getMotorVoltage()),
                new LoggedStatusSignal("Steer Supply Current", steerMotor.getSupplyCurrent()),


                new LoggedStatusSignal("Drive Position", driveMotor.getPosition()),
                new LoggedStatusSignal("Drive Rotor Position", driveMotor.getRotorPosition()),
                new LoggedStatusSignal("Drive Velocity", driveMotor.getVelocity()),
                new LoggedStatusSignal("Drive Stator Current", driveMotor.getStatorCurrent()),
                new LoggedStatusSignal("Drive Voltage", driveMotor.getMotorVoltage()),
                new LoggedStatusSignal("Drive Supply Current", driveMotor.getSupplyCurrent()),


                new LoggedStatusSignal("Encoder Position", steerEncoder.getPosition()),
                new LoggedStatusSignal("Encoder Velocity", steerEncoder.getVelocity())
        };


    }

    @Override
    public void periodic() {

        Logger.recordOutput("Chassis/steer/position", steerPosition.getValue());
        Logger.recordOutput("Chassis/steer/velocity", steerVelocity.getValue());
        Logger.recordOutput("Chassis/steer/current", steerSupplyCurrent.getValue());
        Logger.recordOutput("Chassis/steer/voltage", steerVoltage.getValue());

        Logger.recordOutput("Chassis/drive/position", drivePosition.getValue());
        Logger.recordOutput("Chassis/drive/rotorPosition", driveRotorPosition.getValue());
        Logger.recordOutput("Chassis/drive/velocity", driveVelocity.getValue());
        Logger.recordOutput("Chassis/drive/current", driveStatorCurrent.getValue());
        Logger.recordOutput("Chassis/drive/voltage", driveVoltage.getValue());

        Logger.recordOutput("Chassis/encoder/position",  encoderPosition.getValue());
        Logger.recordOutput("Chassis/encoder/velocity", encoderVelocity.getValue());



    }

}