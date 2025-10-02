package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final String moduleName;
    SwerveModule(String moduleName, int driveMotorId, int steerMotorId, int encoderId) {
        this.moduleName = moduleName;
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);

        driveMotor.getConfigurator().apply(ChassisConstants.createDriveMotorConfiguration(encoderId));
        steerMotor.getConfigurator().apply(ChassisConstants.createSteerMotorConfiguration( encoderId));
    }
}
