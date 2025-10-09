package org.tahomarobotics.robot.Chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;



class SwerveModule extends SubsystemBase {

    //Hardware
    private final TalonFX driveMotor;
    private final  TalonFX steerMotor;
    private final CANcoder steerEncoder;

    //Identifiers
    private final String name;
    public static final String CANBUS_NAME = "Canivore";/* should be in RobotConfiguration */

    //Initialization

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

//    private final  StatusSignal<AngularVelocity> driveVel;
//    private final StatusSignal<Angle> steerPos;
//    private final StatusSignal<AngularVelocity> steerVel;

    //Constructors
    public SwerveModule(RobotMap.moduleId descriptor, double angularOffset) {
        name = descriptor.moduleName();

        driveMotor = new TalonFX(descriptor.driveId(), CANBUS_NAME);

        steerMotor = new TalonFX(descriptor.steerId(), CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.cancoderId(), CANBUS_NAME);
        RobustConfigurator.tryConfigureTalonFX(" Drive Motor", driveMotor, createDriveMotorConfiguration());
        RobustConfigurator.tryConfigureTalonFX(" Steer Motor", steerMotor, createSteerMotorConfiguration());
        RobustConfigurator.tryConfigureCANcoder(" Encoder ", steerEncoder, createEncoderConfiguration(angularOffset));

        Logger.info("Creating an instance of SwerveModule: Drive Motor, Steer Motor, and Encoder " );

//        driveVel = driveMotor.getVelocity();
//        new LoggedStatusSignal("drive Speed" , driveVel);
//
//        steerPos = steerEncoder.getAbsolutePosition();
//        steerVel = steerEncoder.getVelocity();
    }

//    public double getSteerAngle() {
//        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPos, steerVel);
//    }

//    //getter
//    public SwerveModuleState getState() {
//        return new SwerveModuleState(driveVel.getValueAsDouble(), Rotation2d.fromRotations(getSteerAngle()));
//    }
    
//    //setter
//    public void setState(SwerveModuleState state) {
//        double steerAngle = getSteerAngle();
//        targetState = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(state.angle.getDegrees()));
//        driveMotor.setControl(driveMotorVelocity.withVelocity(targetState.speedMetersPerSecond /* should be 'targetState.speedMetersPerSecond/DRIVE_POSITION_COEFFICIENT' but to do that I would need to start doing chassis constants too */ ));
//        steerMotor.setControl(steerMotorPosition.withPosition(targetState.angle.getRotations()));
//    }

//    // Calibration
//    public void calibration () {
//        RobustConfigurator.trySetCANcoderAngularOffset(name + " Encoder ", steerEncoder, 0.0);
//        RobustConfigurator.trySetMotorNeutralMode(name + " Steer Motor ", steerMotor, NeutralModeValue.Coast);
//    }

}
