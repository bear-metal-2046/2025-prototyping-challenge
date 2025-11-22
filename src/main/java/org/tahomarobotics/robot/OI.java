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

package org.tahomarobotics.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.endeffector.EndEffector;


import static edu.wpi.first.units.Units.Volts;

public class OI extends SubsystemBase {
    private static final int DRIVER_CONTROLLER_INDEX = 0;
    private final EndEffector endEffector;
    private final Elevator elevator;
    private final Arm arm;
    private final Windmill windmill;
    private static final double DEADBAND = 0.09;
    private static final double DESENSITIZED_POWER = 1.5;

    final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_INDEX);

    public void configureBindings() {
        endEffector.setDefaultCommand(endEffector.applyVoltage(() -> Volts.of(getLeftTrigger())));

        // windmill low position
        driverController.a().onTrue(windmill.moveWindmill(WindmillPosition.TeamPositions.TEAM_POSITIONS.getLow()));

        // windmill mid position
        driverController.b().onTrue(windmill.moveWindmill(WindmillPosition.TeamPositions.TEAM_POSITIONS.getMid()));

        // windmill high position
        driverController.y().onTrue(windmill.moveWindmill(WindmillPosition.TeamPositions.TEAM_POSITIONS.getHigh()));

        // windmill stow position
        driverController.x().onTrue(windmill.moveWindmill(WindmillPosition.TeamPositions.TEAM_POSITIONS.getHigh()));

    }


    public double getLeftTrigger() {
        return desensitizePowerBased(driverController.getLeftTriggerAxis(), DESENSITIZED_POWER);
    }

    public OI(RobotContainer robotContainer) {

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
        this.endEffector = robotContainer.endEffector;
        this.elevator = robotContainer.elevator;
        this.arm = robotContainer.arm;
        this.windmill = robotContainer.windmill;

        configureBindings();
    }

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}
