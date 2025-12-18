package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
//        this.registerTelemetry(this::telemeterize);
    }

    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
                ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE, Degrees.of(-18.2)),
                ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE, Degrees.of(-28.2)),
                ChassisConstants.getModuleConfig(BACK_LEFT_MODULE, Degrees.of(-71.9)),
                ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE, Degrees.of(-137.7) )
        );
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }

    @Override
    public void periodic() {
        var state = this.getState();
        Logger.recordOutput("Chassis/SwerveStates" , state.ModuleStates);
    }

    @Override
    public void close() {}
}
