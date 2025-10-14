package org.tahomarobotics.robot.Chassis;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tinylog.Logger;

import static org.tahomarobotics.robot.RobotMap.CANBUS_NAME;


class SwerveModule extends SubsystemBase {

    //Hardware
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    //Identifiers
    private final String name;


    private SwerveModuleState targetState = new SwerveModuleState();

    //Constructors
    public SwerveModule(RobotMap.moduleId descriptor) {
        name = descriptor.moduleName();

        driveMotor = new TalonFX(descriptor.driveId(), CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.cancoderId(), CANBUS_NAME);

        Logger.info("Creating an instance of SwerveModule: Drive Motor, Steer Motor, and Encoder ");

    }
}