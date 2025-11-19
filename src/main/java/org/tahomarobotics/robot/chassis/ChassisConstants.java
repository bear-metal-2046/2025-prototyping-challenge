package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.tahomarobotics.robot.RobotMap.*;

import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static org.tahomarobotics.robot.RobotMap.*;

public class ChassisConstants {
    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
            new SwerveDrivetrainConstants()
                    .withCANBusName(CANBUS_NAME)
                    .withPigeon2Id(PIGEON);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
//                    .withDriveMotorGearRatio(kDriveGearRatio)
//                    .withSteerMotorGearRatio(kSteerGearRatio)
//                    .withCouplingGearRatio(kCoupleRatio)
//                    .withWheelRadius(kWheelRadius)
//                    .withSteerMotorGains(steerGains)
//                    .withDriveMotorGains(driveGains)
//                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
//                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
//                    .withSlipCurrent(kSlipCurrent)
//                    .withSpeedAt12Volts(kSpeedAt12Volts)
//                    .withDriveMotorType(kDriveMotorType)
//                    .withSteerMotorType(kSteerMotorType)
//                    .withFeedbackSource(kSteerFeedbackType)
//                    .withDriveMotorInitialConfigs(driveInitialConfigs)
//                    .withSteerMotorInitialConfigs(steerInitialConfigs)
//                    .withEncoderInitialConfigs(encoderInitialConfigs)
//                    .withSteerInertia(kSteerInertia)
//                    .withDriveInertia(kDriveInertia)
//                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
//                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public static final Distance HALF_TRACK_WIDTH = Meters.of(20.75 / 2.0);
    public static final Distance HALF_WHEELBASE = Meters.of(20.75 / 2.0);

    private static final Map<ModuleId, Distance> DISTANCE_X = Map.of(
            FRONT_LEFT_MODULE, HALF_WHEELBASE,
            FRONT_RIGHT_MODULE, HALF_WHEELBASE,
            BACK_LEFT_MODULE, HALF_WHEELBASE.unaryMinus(),
            BACK_RIGHT_MODULE, HALF_WHEELBASE.unaryMinus()
    );

    private static final Map<ModuleId, Distance> DISTANCE_Y = Map.of(
            FRONT_LEFT_MODULE, HALF_TRACK_WIDTH.unaryMinus(),
            FRONT_RIGHT_MODULE, HALF_TRACK_WIDTH,
            BACK_LEFT_MODULE, HALF_TRACK_WIDTH.unaryMinus(),
            BACK_RIGHT_MODULE, HALF_TRACK_WIDTH
    );

    public static SwerveModuleConstants <TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getModuleConfig(ModuleId moduleId, Angle steerOffset) {
        return MODULE_CONSTANTS_FACTORY
                .createModuleConstants(moduleId.steerId(),
                        moduleId.driveId(),
                        moduleId.cancoderId(),
                        steerOffset,
                        DISTANCE_X.get(moduleId),
                        DISTANCE_Y.get(moduleId),
                        false, false, false);
        }
}


