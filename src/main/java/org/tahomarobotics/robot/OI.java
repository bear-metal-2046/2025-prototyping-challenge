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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.endeffector.EndEffector;
import org.tinylog.Supplier;

import static edu.wpi.first.units.Units.Volts;

public class OI extends SubsystemBase {

    private static final int DRIVER_CONTROLLER_INDEX = 0;
    public EndEffector endEffector = new EndEffector();
    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double DESENSITIZED_POWER =1.5;

    final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_INDEX);




    public void ConfigureBinding() {
       endEffector.setDefaultCommand(endEffector.applyVoltage(()-> Volts.of(getLeftTrigger())));


    }
    public double getLeftTrigger() {
        return desensitizePowerBased(driverController.getLeftTriggerAxis(), DESENSITIZED_POWER);
    }



    public OI(RobotContainer robotContainer) {

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());

        ConfigureBinding();
    }
    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }



}
