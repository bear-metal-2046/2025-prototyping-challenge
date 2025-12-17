/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.tahomarobotics.robot.RobotMap.*;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class ChassisSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    private final ChassisSimulation simulation;
    private boolean m_hasAppliedOperatorPerspective;

    public ChassisSubsystem(DeviceConstructor<TalonFX> driveMotorConstructor,
            DeviceConstructor<TalonFX> steerMotorConstructor,
            DeviceConstructor<CANcoder> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveMotorConstructor, steerMotorConstructor, encoderConstructor, drivetrainConstants, modules);

        simulation = new ChassisSimulation(getPigeon2(), getModules());
    }

    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
                ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE, Degrees.of(0d)),
                ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE, Degrees.of(0d)),
                ChassisConstants.getModuleConfig(BACK_LEFT_MODULE, Degrees.of(0d)),
                ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE, Degrees.of(0d)));
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        var state = getStateCopy();
        //Logger.recordOutput("Drive/OdometryPose", state.Pose);
        Logger.recordOutput("Drive/TargetStates", state.ModuleTargets);
        Logger.recordOutput("Drive/MeasuredStates", state.ModuleStates);
        Logger.recordOutput("Drive/MeasuredSpeeds", state.Speeds);
    }

    public ChassisSimulation getSimulation() {
        return simulation;
    }

    /**
     * Resets the robot pose along with simulated pose
     */
    public void setPose(Pose2d pose) {
        super.resetPose(pose);

        if (Robot.isSimulation()) {
            simulation.setPose(pose);
        }
    }
}
