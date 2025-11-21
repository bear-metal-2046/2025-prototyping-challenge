package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static org.tahomarobotics.robot.RobotMap.*;

public class ChassisSubsystem extends SwerveDrivetrain<TalonFX,TalonFX, CANcoder> implements AutoCloseable, Subsystem {
   @AutoLogOutput (key = "ChassisSpeeds")
    private ChassisSpeeds speeds = new ChassisSpeeds();


    public ChassisSubsystem(DeviceConstructor<TalonFX> driveMotorConstructor,
                            DeviceConstructor<TalonFX> steerMotorConstructor,
                            DeviceConstructor<CANcoder> encoderConstructor,
                            SwerveDrivetrainConstants drivetrainConstants,
                            SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveMotorConstructor, steerMotorConstructor, encoderConstructor, drivetrainConstants, modules);
    }

    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
                ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE, Degrees.of(-18.2d)),
                ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE, Degrees.of(-28.2d)),
                ChassisConstants.getModuleConfig(BACK_LEFT_MODULE, Degrees.of(-71.9d)),
                ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE, Degrees.of(-132.7d))
        );
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }

    @Override
    public void close() {}

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput("SwerveModuleStates/" + i, this.getModule(i).getCurrentState());
        }
    }
}
