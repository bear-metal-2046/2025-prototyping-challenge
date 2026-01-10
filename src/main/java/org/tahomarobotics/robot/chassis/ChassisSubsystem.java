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

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.tahomarobotics.robot.RobotMap.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotMap.ModuleId;

import static edu.wpi.first.units.Units.*;

public class ChassisSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    private static final ModuleId[] MODULE_IDS = new ModuleId[] {
            FRONT_LEFT_MODULE,
            FRONT_RIGHT_MODULE,
            BACK_LEFT_MODULE,
            BACK_RIGHT_MODULE
    };

    private final ChassisSimulation simulation;
    private boolean isOperatorPerspectiveApplied;

    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
                ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE),
                ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE),
                ChassisConstants.getModuleConfig(BACK_LEFT_MODULE),
                ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE));
    }

    ChassisSubsystem(DeviceConstructor<TalonFX> driveMotorConstructor,
            DeviceConstructor<TalonFX> steerMotorConstructor,
            DeviceConstructor<CANcoder> encoderConstructor,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveMotorConstructor, steerMotorConstructor, encoderConstructor, drivetrainConstants, modules);

        simulation = Robot.isSimulation() ? new ChassisSimulation(getPigeon2(), getModules()) : null;
    }

    @Override
    public void periodic() {
        if (!isOperatorPerspectiveApplied || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                isOperatorPerspectiveApplied = true;
            });
        }

        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        var state = getStateCopy();
        Logger.recordOutput("Drive/OdometryPose", state.Pose);
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

    Angle[] oldOffsets = new Angle[getModules().length];
    private final static Angle[] ZERO_OFFSETS = new Angle[] {
            Degrees.of(0.0),
            Degrees.of(0.0),
            Degrees.of(0.0),
            Degrees.of(0.0)
    };

    public void initAlign() {
        coast();
        oldOffsets = getAndSetOffset(ZERO_OFFSETS);

        System.out.println("Aligning Swerve Modules");
        org.tinylog.Logger.info("Aligning Swerve Modules");
    }

    private void cancelAlign() {
        brake();
        for (var moduleNum = 0; moduleNum < getModules().length; moduleNum++) {
            getAndSetOffset(oldOffsets);
        }
        org.tinylog.Logger.info("Cancelling Swerve Module ReAlignment");
        System.out.println("Align Cancelled");
        /*
         * Restore offsets
         * Reset coast
         */
    }

    private void completeAlign() {
        /*
         * Get current angles and negate as new offsets
         * Save new offsets in preferences
         * Apply new offsets
         * Reset coast
         */
        brake();
        Angle[] newOffsets = new Angle[getModules().length];
        for (var moduleNum = 0; moduleNum < getModules().length; moduleNum++) {
            newOffsets[moduleNum] = getModule(moduleNum).getCurrentState().angle.getMeasure().unaryMinus();
            Preferences.setDouble(ChassisConstants.getModuleOffKey(MODULE_IDS[moduleNum]),
                    newOffsets[moduleNum].in(Degrees));
            getAndSetOffset(newOffsets);
        }
        org.tinylog.Logger.info("Swerve Modules Aligned");
        System.out.println("Swerve Modules Aligned");
    }

    public Consumer<Boolean> finishAlign() {
        return (canceled) -> {
            if (canceled) {
                cancelAlign();
            } else {
                completeAlign();
            }
        };
    }

    public void coast() {
        for (var module = 0; module < getModules().length; module++) {
            getModule(module).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        }
    }

    public void brake() {
        for (var module = 0; module < getModules().length; module++) {
            getModule(module).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        }
    }

    public Angle[] getAndSetOffset(Angle[] newOffsets) {
        MagnetSensorConfigs cfg = new MagnetSensorConfigs();
        int moduleCount = getModules().length;
        Angle[] offsets = new Angle[moduleCount];
        for (var moduleNum = 0; moduleNum < moduleCount; moduleNum++) {
            var configurator = getModule(moduleNum).getEncoder().getConfigurator();
            // read current offset
            configurator.refresh(cfg);
            offsets[moduleNum] = cfg.getMagnetOffsetMeasure();
            // set new offset
            cfg.withMagnetOffset(newOffsets[moduleNum]);
            configurator.apply(cfg);

        }
        return offsets;
    }

    // modules encoder config refresh
    //
    //
    // public void newOffsets() {
    // for (var module : getModules()) {
    // Preferences.setDouble(module+ "Offset",
    // module.getCurrentState().angle.getDegrees() * -1);
    //
    // }
}
