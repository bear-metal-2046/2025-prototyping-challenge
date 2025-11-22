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
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.chassis.Chassis;
import edu.wpi.first.wpilibj.Preferences;

public class OI {
    private static final int DRIVER_CONTROLLER_INDEX = 0;
    final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_INDEX);

    public OI(RobotContainer robotContainer) {
        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
        Chassis chassis = robotContainer.chassis;
        chassis.setDefaultCommand(chassis.teleopDrive(driverController::getLeftY, driverController::getLeftX, driverController::getRightX));
        chassis.postZeroSteersCommand();
        driverController.y().onTrue(Commands.run(() -> {chassis.constantDrive(0.0, 1.0, 0.0);}));
        driverController.x().onTrue(Commands.run(() -> {chassis.constantDrive(0.0, -1.0, 0.0);}));
        driverController.b().onTrue(Commands.runOnce(chassis::zeroSteers));
    }
}
