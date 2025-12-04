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
        this.registerTelemetry(this::telemeterize);
    }

    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
                ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE),
                ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE),
                ChassisConstants.getModuleConfig(BACK_LEFT_MODULE),
                ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE)
        );
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.Position));
    }

    public void zeroSteers() {
        for (int i = 0; i < 4; i++) {
            Angle offset = Degrees.of(-getModule(i).getCurrentState().angle.getDegrees());

            // Find the correct name and config for the module so we send it to preferences correctly
            String name;
            MagnetSensorConfigs config;
            switch (i) {
                case 0 -> {
                    name = FRONT_LEFT_MODULE.moduleName() + "Offset";
                    config = ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE).EncoderInitialConfigs.MagnetSensor;
                }
                case 1 -> {
                    name = FRONT_RIGHT_MODULE.moduleName() + "Offset";
                    config = ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE).EncoderInitialConfigs.MagnetSensor;
                }
                case 2 -> {
                    name = BACK_LEFT_MODULE.moduleName() + "Offset";
                    config = ChassisConstants.getModuleConfig(BACK_LEFT_MODULE).EncoderInitialConfigs.MagnetSensor;
                }
                case 3 -> {
                    name = BACK_RIGHT_MODULE.moduleName() + "Offset";
                    config = ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE).EncoderInitialConfigs.MagnetSensor;
                }
                default -> {
                    name = "oops, something went wrong.";
                    config = ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE).EncoderInitialConfigs.MagnetSensor;

                }
            }

            config = config.withMagnetOffset(offset.plus(Rotations.of(config.MagnetOffset)));

            org.tinylog.Logger.info(name + " " + offset.in(Degrees) + " degrees");
            // Write to the offsets file and apply new config to encoder.
            Preferences.setDouble(name, offset.in(Degrees));
        }
    }

    /*TEMPORARY FOR TUNING, UNREFACTORED FROM CTRE*/



    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * ChassisConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond)));
        }
    }

    @Override
    public void periodic() {
        this.getState();
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput("SwerveModuleStates/" + i, this.getModule(i).getCurrentState());

            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS DRIVE PROPORTIONAL"+ i, getModule(i).getDriveMotor().getClosedLoopProportionalOutput().getValue());
            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS STEER PROPORTIONAL"+ i, getModule(i).getSteerMotor().getClosedLoopProportionalOutput().getValue());

            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS DRIVE INTEGRATED"+ i, getModule(i).getDriveMotor().getClosedLoopIntegratedOutput().getValue());
            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS STEER INTEGRATED"+ i, getModule(i).getSteerMotor().getClosedLoopIntegratedOutput().getValue());

            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS DRIVE DERIVATIVE"+ i, getModule(i).getDriveMotor().getClosedLoopDerivativeOutput().getValue());
            Logger.recordOutput("POOP AHH CLOSED LOOP OUTPUTS STEER DERIVATIVE"+ i, getModule(i).getSteerMotor().getClosedLoopDerivativeOutput().getValue());
        }
    }

    @Override
    public void close() {}
}
