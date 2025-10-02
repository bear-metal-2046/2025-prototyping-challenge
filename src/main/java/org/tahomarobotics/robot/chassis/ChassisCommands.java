package org.tahomarobotics.robot.chassis;

public class ChassisCommands {

    public static SwerveDriveCommand swerveDriveCommand(SwerveModule swerveModule, double driveOutput, double steerOutput) {
        return new SwerveDriveCommand(swerveModule, driveOutput, steerOutput);
    }
}
