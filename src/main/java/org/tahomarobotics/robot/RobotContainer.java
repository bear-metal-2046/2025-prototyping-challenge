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

import org.tahomarobotics.robot.endeffector.EndEffector;

import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.sim.Simulation;


/**
 * The RobotContainer declares the robot's structure: subsystems, commands and simulations.
 * All the robot's components and their relationships are declared here rather than scattered
 * throughout the codebase.
 */
public class RobotContainer implements AutoCloseable {

    // Subsystems will be declared here as public final fields, including:
    // - Chassis (drive subsystem)
    public final Chassis chassis;
    // - Elevator
    // - Arm
    // - End effector
    // - Vision
    // - Robot visualization and simulation components
    public final Elevator elevator;
    public final Arm arm;
    public final EndEffector endEffector;
    public final Simulation simulation;

    public RobotContainer() {
        // NetworkTables must be loaded for Preferences to work correctly.
        // By default, it doesn't load until after this constructor is called, 
        // so we want to force it to load here.


        // Arena must be created first to override the default instance provided by
        // org.ironmaple.simulation.SimulatedArena.getInstance() which is used by MapleSim
        //Arena2025Reefscape arena = org.tahomarobotics.sim.Arena2025Reefscape.getInstance();

        // Create subsystem instances
        chassis = new Chassis();
        // Create subsystem instances here
        elevator = new Elevator();
        arm = new Arm();
        endEffector = new EndEffector();

        simulation = new Simulation(arm.getSimulation());

        OI oi = new OI(this);
    }

    @Override
    public void close() {}
}