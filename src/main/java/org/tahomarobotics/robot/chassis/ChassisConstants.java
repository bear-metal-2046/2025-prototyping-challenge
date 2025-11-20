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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.tahomarobotics.robot.RobotMap.*;

public class ChassisConstants {

    public static final double DRIVE_GEAR_RATIO = (54d / 14d) * (25d / 32d) * (30d / 15d);
    public static final double STEER_GEAR_RATIO = (49d / 7d) * (41d / 11d);
    public static final double COUPLING_GEAR_RATIO = 54d / 14d;
    public static final Distance WHEEL_RADIUS = Inches.of(2d);

    private static final SwerveModuleConstants.ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    private static final SwerveModuleConstants.ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
    private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;
    private static final SwerveModuleConstants.SteerFeedbackType STEER_FEEDBACK_TYPE = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;
    private static final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
            new SwerveDrivetrainConstants()
                    .withCANBusName(CANBUS_NAME)
                    .withPigeon2Id(PIGEON);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                  .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                  .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                  .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                  .withWheelRadius(WHEEL_RADIUS)
//                    .withSteerMotorGains(steerGains)
//                    .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                  .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
//                    .withSlipCurrent(kSlipCurrent)
//                    .withSpeedAt12Volts(kSpeedAt12Volts)
                  .withDriveMotorType(DRIVE_MOTOR_TYPE)
                  .withSteerMotorType(STEER_MOTOR_TYPE)
                    .withFeedbackSource(STEER_FEEDBACK_TYPE)
                  .withDriveMotorInitialConfigs(DRIVE_MOTOR_CONFIG)
                  .withSteerMotorInitialConfigs(STEER_MOTOR_CONFIG)
                  .withEncoderInitialConfigs(CANCODER_CONFIG);
//                    .withSteerInertia(kSteerInertia)
//                    .withDriveInertia(kDriveInertia)
//                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
//                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public static final Distance HALF_TRACK_WIDTH = Meters.of(20.75d / 2d);
    public static final Distance HALF_WHEELBASE = Meters.of(20.75d / 2d);

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


