package org.tahomarobotics.robot.Chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;

class SwerveModule extends SubsystemIF {

    private final TalonFX driveMotor;
    private final  TalonFX steerMotor;
    private final CANcoder steerEncoder;

    public static final String CANBUS_NAME = "Canivore";
    public static TalonFXConfiguration createDriveMotorConfiguration() {
        return new TalonFXConfiguration();
    }
    public static TalonFXConfiguration createSteerMotorConfiguration() {
        return new TalonFXConfiguration();
    }

    public static CANcoderConfiguration createEncoderConfiguration(double angularOffset) {
        return new CANcoderConfiguration();
    }


    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(true);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(true);

    private SwerveModuleState targetState = new SwerveModuleState();

    private final  StatusSignal<AngularVelocity> driveVel;
    private final StatusSignal<Angle> steerPos;
    private final StatusSignal<AngularVelocity> steerVel;

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        driveMotor = new TalonFX(descriptor.driveId(), CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerID(), CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.encoderID(), CANBUS_NAME);
        RobustConfigurator.tryConfigureTalonFX(" Drive Motor", driveMotor, createDriveMotorConfiguration());
        RobustConfigurator.tryConfigureTalonFX(" Steer Motor", steerMotor, createSteerMotorConfiguration());
        RobustConfigurator.tryConfigureCANcoder(" Encoder ", steerEncoder, createEncoderConfiguration(angularOffset));
        driveVel = driveMotor.getVelocity();
        new LoggedStatusSignal("drive Speed" , driveVel);

        steerPos = steerEncoder.getAbsolutePosition();
        steerVel = steerEncoder.getVelocity();
    }

    public double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPos, steerVel);
    }


    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVel.getValueAsDouble(), Rotation2d.fromRotations(getSteerAngle()));
    }

    public void setState(SwerveModuleState state) {
        double steerAngle = getSteerAngle();
        targetState = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(state.angle.getDegrees()));
        driveMotor.setControl(driveMotorVelocity.withVelocity(targetState.speedMetersPerSecond /* should be over DRIVE_POSITION_COEFFICIENT */ ));
        steerMotor.setControl(steerMotorPosition.withPosition(targetState.angle.getRotations()));
    }

}
